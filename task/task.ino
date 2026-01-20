
#include <Arduino.h>

// -------------------- TUNING (START VALUES) --------------------
  int delaySet  = 0;
  int baseSpeed = 250;          // start lower while tuning

  // Keep Ki = 0 for digital sensors until everything else is stable
  float Ki = 0.0;

  // Anti-windup (mostly irrelevant with Ki=0, but kept for later)
  float integralLimit = 200.0f;

  // Weights for 5 sensors
  int weights[5] = {2, 1, 0, -1, -2};

  // Digital threshold
  int threshold = 700;                  // adjust 1100–1600 if needed
  const bool SENSOR_HIGH_ON_LINE = true; // RAW HIGH when on line (your sensor ranges)

  // Corner slow-down
  int slowDown1 = 40;          // when abs(error)==1 (inner sensors active)
  int slowDown2 = 150;          // when abs(error)>=2 (outer sensors active)

  // Gain scheduling (soft in center, stronger in corners)
  float Kp_center = 28.0f;     // used when absErr<=1
  float Kd_center = 12.0f;

  float Kp_corner = 40.0f;     // used when absErr>=2
  float Kd_corner = 24.0f;

  // Turn clamp scheduling
  float maxTurn_center = 95.0f;
  float maxTurn_corner = 350.0f;

  // Derivative smoothing (less laggy than 0.95 for digital steps)
  const float dAlpha_use = 0.88f;

  // Turn slew-rate limit (prevents sharp snapping)
  float turnSlewRate = 350.0f; // turn units per second (tune 150..400)

  // --- NEW: Inner sensor softness (abs(error)==1) ---
  float innerErrorScale = 0.60f;  // soft straighten-up strength (tune 0.15..0.60)

  // Optional: also soften D when absErr==1 (helps remove twitch)
  float innerDScale = 0.60f;      // tune 0.40..1.00

// -------------------- PINS --------------------
  int motor1PWM   = 37;  // Right motor PWM
  int motor1Phase = 38;  // Right motor direction
  int motor2PWM   = 39;  // Left motor PWM
  int motor2Phase = 20;  // Left motor direction

  const int N = 5;
  int AnalogPin[N] = {4, 5, 6, 7, 15};

  int AnalogValue[N]  = {0,0,0,0,0};
  int DigitalValue[N] = {0,0,0,0,0};

// -------------------- PID STATE --------------------
  float integral = 0.0f;
  float lastEUse = 0.0f;     // store EFFECTIVE error used by PID
  float dFilt = 0.0f;
  float lastTurn = 0.0f;
  unsigned long lastTimeMs = 0;

// -------------------- Clamp --------------------
  int clamp255(int v) {
    if (v < 0) return 0;
    if (v > 255) return 255;
    return v;
  }

// -------------------- MOTOR CONTROL --------------------
  void rightFoward(int speed){
    digitalWrite(motor1Phase, HIGH);
    analogWrite(motor1PWM, speed);
  }
  void rightReverse(int speed){
    digitalWrite(motor1Phase, LOW);
    analogWrite(motor1PWM, abs(speed));
  }
  void leftFoward(int speed){
    digitalWrite(motor2Phase, LOW);
    analogWrite(motor2PWM, speed);
  }
  void leftReverse(int speed){
    digitalWrite(motor2Phase, HIGH);
    analogWrite(motor2PWM, abs(speed));
  }

  // Forward-only (stable for PID)
  void drive(int rightSpeed, int leftSpeed) {
    if (rightSpeed>0){rightFoward(rightSpeed);}
      else{rightReverse(rightSpeed);}
    if (leftSpeed>0){leftFoward(leftSpeed);}
      else{leftReverse(leftSpeed);}
  }

// -------------------- READ DIGITAL ERROR --------------------

    int sensorToDigital(int raw) {
    // Returns 1 if sensor "sees the line", else 0
    if (SENSOR_HIGH_ON_LINE) return (raw > threshold) ? 1 : 0;
    return (raw < threshold) ? 1 : 0;
  }

  void readSensor(){
    for (int i = 0; i < 5; i++) {
      DigitalValue[i] = sensorToDigital(analogRead(AnalogPin[i]));
    }
  }
  /*
    Threshold sensors into 0/1, then:
      error = Σ(DigitalValue[i] * weights[i])

    lineLost => no sensors see the line (sumOn == 0)
  */

  float readLineErrorDigital(bool &lineLostOut) {
    int sumOn = 0;
    int e = 0;

    for (int i = 0; i < N; i++) {
      int raw = analogRead(AnalogPin[i]);
      AnalogValue[i] = raw;

      DigitalValue[i] = sensorToDigital(raw);
      sumOn += DigitalValue[i];

      e += DigitalValue[i] * weights[i];
    }

    lineLostOut = (sumOn == 0);    

    // If lost, keep lastEUse sign so recovery turns the right way
    if (lineLostOut) return lastEUse;

    return (float)e;
  }

// -------------------- SETUP --------------------
  void setup() {
    Serial.begin(9600);

    pinMode(motor1PWM, OUTPUT);
    pinMode(motor1Phase, OUTPUT);
    pinMode(motor2PWM, OUTPUT);
    pinMode(motor2Phase, OUTPUT);

    analogReadResolution(12);        // 0..4095
    analogSetAttenuation(ADC_11db);  // best for ~0..3.3V

    analogWrite(motor1PWM, 0);
    analogWrite(motor2PWM, 0);

    lastTimeMs = millis();
    }
//--------------Loop----------------
  void loop(){
    /*if (detectNode()){
      drive(0,0);
      turnRight();
      follow();
      }
    else{follow();}*/
    follow();
    }


// -------------------- follow line --------------------
  void follow() {
    delay(delaySet);

    // 1) Read digital line error
    bool lineLost = false;
    float error = readLineErrorDigital(lineLost);

    // 2) Compute dt
    unsigned long now = millis();
    float dt = (now - lastTimeMs) / 1000.0f;
    if (dt <= 0.005f) dt = 0.005f;
    lastTimeMs = now;

    // 3) Line lost recovery: arc-spin based on lastEUse sign (forward-only)
    if (lineLost) {
      // Quick spin-search to reacquire (stronger than arcing)
      int search = 120;            // tune 90..160
      int base   = baseSpeed - 40; // slow a bit while searching

      if (lastEUse >= 0) drive(base - search, base + search);
      else               drive(base + search, base - search);

      // Don’t run PID this cycle
      return;
    }

    // 4) Corner slow-down
    int absErr = abs((int)error);
    int localBase = baseSpeed;
    if (absErr >= 2) localBase = baseSpeed - slowDown2;
    else if (absErr == 1) localBase = baseSpeed - slowDown1;

    // 5) INNER-SOFT / OUTER-STRONG: shape effective error
    float eUse = error;
    if (absErr == 1) {
      // inner sensors: gentle straighten-up
      eUse = innerErrorScale * error;
    }
    // absErr>=2 keeps full error (outer sensors strong)

    // 6) PID core (integral, derivative computed on eUse)
    integral += eUse * dt;
    if (integral >  integralLimit) integral =  integralLimit;
    if (integral < -integralLimit) integral = -integralLimit;

    float dRaw = (eUse - lastEUse) / dt;
    lastEUse = eUse;

    // Derivative smoothing
    dFilt = dAlpha_use * dFilt + (1.0f - dAlpha_use) * dRaw;

    // Gain scheduling: softer in center, stronger in corners (based on ORIGINAL absErr)
    float Kp_use = (absErr >= 2) ? Kp_corner : Kp_center;
    float Kd_use = (absErr >= 2) ? Kd_corner : Kd_center;

    // Optional: further soften D when inner sensors are active (reduces twitch)
    if (absErr == 1) Kd_use *= innerDScale;

    float turn = (Kp_use * eUse) + (Ki * integral) + (Kd_use * dFilt);

    // 7) Variable maxTurn: smaller on straights, larger in corners
    float maxTurn_use = (absErr >= 2) ? maxTurn_corner : maxTurn_center;
    if (turn >  maxTurn_use) turn =  maxTurn_use;
    if (turn < -maxTurn_use) turn = -maxTurn_use;

    // 8) Turn slew-rate limiting (prevents sharp oscillation/snapping)
    float maxTurnStep = turnSlewRate * dt;
    float delta = turn - lastTurn;
    if (delta >  maxTurnStep) turn = lastTurn + maxTurnStep;
    if (delta < -maxTurnStep) turn = lastTurn - maxTurnStep;
    lastTurn = turn;

    // 9) Convert to motor speeds
    int rightSpeed = (int)(localBase - turn);
    int leftSpeed  = (int)(localBase + turn);

    // 10) Drive
    rightSpeed=clamp255(rightSpeed);
    leftSpeed=clamp255(leftSpeed);
    drive(rightSpeed, leftSpeed);
  }

//------turn at node----
  bool detectNode(){
    readSensor();
    return ((DigitalValue[0]==0)&&(DigitalValue[4]==0));
    }
  void turn180(){
    drive(-255,255);
    delay (550);
    while (true){
      readSensor();
      if (DigitalValue[2]==0){
        break;
      }
    }
    drive (0,0);
    delay(100);

    integral = 0.0f;
    lastEUse = 0.0f;     // reset pid EFFECTIVE error used by PID
    dFilt = 0.0f;
    lastTurn = 0.0f;
    lastTimeMs = millis();
  }
  void turnLeft(){
      drive(255,-255);
      delay (200);
      while (true){
        readSensor();
        if (DigitalValue[2]==0){
          break;
          }
        }
      drive (0,0);
      delay(100);
      integral = 0.0f;
      lastEUse = 0.0f;     // reset pid EFFECTIVE error used by PID
      dFilt = 0.0f;
      lastTurn = 0.0f;
      lastTimeMs = millis();

      }
      void turnRight(){
        drive(255,-255);
        delay (200);
        while (true){
          readSensor();
          if (DigitalValue[2]==0){
            break;
          }
        }
        drive (0,0);
        delay(100);

        integral = 0.0f;
        lastEUse = 0.0f;     // reset pid EFFECTIVE error used by PID
        dFilt = 0.0f;
        lastTurn = 0.0f;
        lastTimeMs = millis();

      }

//-----drive to neighbouring node--------
  void followNode(int to){
    if (prev==to){turn180();}
    while (true){
          follow();
          if (detectNode()){
            prev = to;
            sendArrival(prev)
            break;
            }
  }
  void driveEdge(int from, int to){
      if (from==0){
        if (to==7){
            followNode(to);
          }
        }
        
      }



  }

//------pathfinding-----
  int position = 0
  int target = 0

  //adjacency mapping
  enum Node { 0, 1, 2, 3, 4, 5, 6, 7, NODE_COUNT };

  struct Edge { uint8_t to; uint8_t w; }; // path to node (to) had weight (w)
      // array of all edges adjacent to nodes 0,1,2...
  const Edge adj6[]  = { {3,2}, {4,2}, {1,1} }; 
  const Edge adj7[]  = { {2,1}, {1,1}, {0,1} };
  const Edge adj0[]  = { {7,1},  {4,1} };
  const Edge adj1[]  = { {7,1}, {6,1}};
  const Edge adj2[]  = { {3,1}, {7,1} };
  const Edge adj3[]  = { {6,2},  {2,1} };
  const Edge adj4[]  = { {6,1},  {0,1} };
  const Edge adj5[]  = {              };

  const Edge* graph[NODE_COUNT] = { adj6, adj7, adj0, adj1, adj2, adj3, adj4 ,adj5};
  const uint8_t deg[NODE_COUNT] = { 3,    3,    2,    1,    2,    2,    2   ,0 };

  // Computes the shortest distance from 'start' to every other node
  // dist[]  -> shortest known distance from start to each node
  // prev[]  -> previous node on the shortest path (for path reconstruction)
  void dijkstra(int start, int dist[NODE_COUNT], int prev[NODE_COUNT]) {

    // Marks whether a node has been permanently processed
    bool used[NODE_COUNT];

    // ---- Initialization ----
    for (int i = 0; i < NODE_COUNT; i++) {
      dist[i] = INF;   // Start with "infinite" distance to all nodes
      prev[i] = -1;    // No previous node yet
      used[i] = false; // No node has been visited
    }

    // Distance from start node to itself is zero
    dist[start] = 0;

    // ---- Main Dijkstra loop ----
    // Runs at most NODE_COUNT times
    for (int iter = 0; iter < NODE_COUNT; iter++) {

      // Find the unused node with the smallest distance
      int u = -1;
      int best = INF;

      for (int i = 0; i < NODE_COUNT; i++) {
        // Choose the closest unvisited node
        if (!used[i] && dist[i] < best) {
          best = dist[i];
          u = i;
        }
      }

      // If no reachable unvisited node remains, stop
      if (u == -1) break;

      // Mark this node as permanently visited
      used[u] = true;

      // ---- Relax all edges from node u ----
      // Look at each neighbor of u
      for (int k = 0; k < deg[u]; k++) {

        int v = graph[u][k].to; // Neighbor node
        int w = graph[u][k].w;  // Weight of edge u -> v

        // If going through u gives a shorter path to v, update it
        if (dist[u] + w < dist[v]) {
          dist[v] = dist[u] + w; // Update shortest distance
          prev[v] = u;           // Remember path: v came from u
        }
      }
    }
  }


    // Builds the shortest path from start -> target using the prev[] array
  // outPath[] will contain the nodes in order: start ... target
  // Returns the number of nodes in the path
  int buildPath(int start, int target, int prev[NODE_COUNT], int outPath[16]) {

    int tmp[16]; // Temporary storage for reversed path
    int n = 0;   // Number of nodes in temporary path

    // ---- Walk backwards from target to start ----
    for (int cur = target; cur != -1 && n < 16; cur = prev[cur]) {
      tmp[n++] = cur;     // Store current node
      if (cur == start)   // Stop once we reach the start
        break;
    }

    // If we never reached the start, there is no valid path
    if (tmp[n - 1] != start)
      return 0; // unreachable

    // ---- Reverse the path so it goes start -> target ----
    int len = 0;
    for (int i = n - 1; i >= 0; i--) {
      outPath[len++] = tmp[i];
    }

    // Return length of the final path
    return len;
  }


