#include <WiFi.h>
#include <HTTPClient.h>
#include <Arduino.h>

//===== function declarations
  bool detectObstacle();
float readDistanceCmOnce();

// ===== Wi-Fi details =====
  const char* ssid     = "iot";
  const char* password = "tiling6whillilew";

  // ===== Server details =====
  const char* server  = "http://3.250.38.184:8000";
  const char* TEAM_ID = "diyh4437";

  // ===== Route (always 6 nodes: 0..5 in some order) =====
  int routeLen =0 ; //
  int routeNodes[20]; //Max size that server can send

  // Parse exactly x comma-separated ints into routeNodes[]
  bool parseRouteDynamic(const String& routeStr, int out[], int& outLen) {
    outLen = 0;
    int start = 0;

    while (start < routeStr.length()) {
      int comma = routeStr.indexOf(',', start);
      if (comma == -1) comma = routeStr.length();

      if (outLen >= 20) return false; // prevent overflow

      String token = routeStr.substring(start, comma);
      token.trim();
      if (token.length() == 0) return false;

      out[outLen++] = token.toInt();
      start = comma + 1;
    }

    return (outLen > 0);
  }


  // ===== Wi-Fi connect =====
  void connectToWiFi() {
    Serial.print("Connecting to network: ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
      Serial.print(".");
      delay(300);
    }

    Serial.println("\nConnected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  }

// ===== GET route =====
  String getRoute() {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi not connected!");
      return "";
    }

    HTTPClient http;
    String url = String(server) + "/api/getRoute/" + TEAM_ID;

    Serial.print("Requesting route from: ");
    Serial.println(url);

    http.begin(url);
    int code = http.GET();

    String body = "";
    if (code > 0) {
      body = http.getString();
      body.trim();
      Serial.print("HTTP Status: ");
      Serial.println(code);
      Serial.print("Route body: ");
      Serial.println(body);
    } else {
      Serial.print("GET failed, error: ");
      Serial.println(code);
    }

    http.end();
    return body;
  }

  // ===== POST arrival =====
  void sendArrival(int position) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi not connected!");
      return;
    }

    HTTPClient http;
    String url = String(server) + "/api/arrived/" + TEAM_ID;
    String postData = "position=" + String(position);

    Serial.println("Sending POST to server...");
    Serial.println(postData);

    http.begin(url);
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");

    int code = http.POST(postData);

    if (code > 0) {
      String response = http.getString();
      response.trim();

      Serial.print("HTTP Status: ");
      Serial.println(code);

      Serial.print("Server response: ");
      Serial.println(response);

      if (response == "Finished") {
        Serial.println("Route complete!");
      } else {
        int nextDestination = response.toInt();
        Serial.print("Next destination: ");
        Serial.println(nextDestination);
      }
    } else {
      Serial.print("POST failed, error: ");
      Serial.println(code);
    }

    http.end();
  }

// -------------------- TUNING (START VALUES) ------------------
  int delaySet  = 0;
  int baseSpeed = 250;          // start lower while tuning

  // Keep Ki = 0 for digital sensors until everything else is stable
  float Ki = 0.0;

  // Anti-windup (mostly irrelevant with Ki=0, but kept for later)
  float integralLimit = 200.0f;

  // Weights for 5 sensors
  int weights[5] = {2, 1, 0, -1, -2};

  // Digital threshold
  int threshold = 800;                  // adjust 1100–1600 if needed
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

// obstacle detect constants 
  enum Node { N0, N1, N2, N3, N4, N5, N6, N7, NODE_COUNT };
  float objThreshold = 4.0;
  const float MAX_VALID_CM = 300.0f;
  bool obstacleFlag = false;
  bool blocked[NODE_COUNT][NODE_COUNT] = {};
  int blockedFrom = -1;
  int blockedTo   = -1;

// -------------------- PINS --------------------
  int motor1PWM   = 37;  // Right motor PWM
  int motor1Phase = 38;  // Right motor direction
  int motor2PWM   = 39;  // Left motor PWM
  int motor2Phase = 20;  // Left motor direction

  int TRIG = 11;
  int ECHO = 12; //distance sensor

  const int N = 5;
  int AnalogPin[N] = {4, 5, 6, 7, 15};

  int DigitalValue[N] = {0,0,0,0,0};

// -------------------- PID STATE --------------------
  float integral = 0.0f;
  float lastEUse = 0.0f;     // store EFFECTIVE error used by PID
  float dFilt = 0.0f;
  float lastTurn = 0.0f;
  unsigned long lastTimeMs = 0;

// -------------------- Clamp 255--------------------
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
      DigitalValue[i] = sensorToDigital(raw);
      sumOn += DigitalValue[i];

      e += DigitalValue[i] * weights[i];
    }

    lineLostOut = (sumOn == 0);    

    // If lost, keep lastEUse sign so recovery turns the right way
    if (lineLostOut) return lastEUse;

    return (float)e;
  }

// -------------------- follow line ,--------------------
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
  bool detectNode() {
    static bool s0_latched = false;
    static bool s4_latched = false;
    static unsigned long t0 = 0;
    static unsigned long t4 = 0;

    const unsigned long WINDOW_MS = 100; // <-- allow sensors to differ by up to 50ms

    readSensor(); // updates DigitalValue[]

    unsigned long now = millis();

    // Latch each sensor when it triggers (goes to 0)
    if (DigitalValue[0] == 0 && !s0_latched) {
      s0_latched = true;
      t0 = now;
    }
    if (DigitalValue[4] == 0 && !s4_latched) {
      s4_latched = true;
      t4 = now;
    }

    // If both latched and close enough in time => node detected
    if (s0_latched && s4_latched && ( (t0 > t4 ? t0 - t4 : t4 - t0) <= WINDOW_MS )) {
      // reset for next detection
      s0_latched = false;
      s4_latched = false;
      return true;
    }

    // Expire latches if the other sensor didn't arrive in time
    if (s0_latched && (now - t0 > WINDOW_MS)) s0_latched = false;
    if (s4_latched && (now - t4 > WINDOW_MS)) s4_latched = false;

    return false;
  }




    
  void turn180(){
    drive(0,0);
    delay(50);
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
      drive(0,0);
      delay(50);
      drive(-255,255);
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
        drive(0,0);
        delay(50);
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
  int previous =4;
  int position=0;
    void followNode(int from,int to){
        if (previous==to){turn180();}
        while (true){
              follow();
              if (detectNode()){
                previous = from;
                position = to;
                sendArrival(position);
                break;
                }
              else if (detectObstacle()){
              delay(1000);
            
              blocked[from][to] = true;
              blocked[to][from] = true;
              blockedFrom = from;
              blockedTo = to;

              Serial.print("Blocked edge: ");
              Serial.print(from);
              Serial.print(" <-> ");
              Serial.println(to);

              turn180();          // you already do this
              obstacleFlag = true;
              return;
            
              }
      }
      }
 void driveEdge(int from, int to) {
    if ((from == 6) && (to == 1)) {
      if (previous == 4) {
        turnRight();
        followNode(from, to);
      } else if (previous == 3) {
        turnLeft();
        followNode(from, to);
      } else {
        drive(255, 255);
        delay(100);
        followNode(from, to);
      }
    }

    else if ((from == 7) && (to == 1)) {
      if (previous == 2) {
        turnRight();
        followNode(from, to);
      } else if (previous == 0) {
        turnLeft();
        followNode(from, to);
      } else {
        drive(255, 255);
        delay(100);
        followNode(from, to);
      }
    }

    else if ((from == 6) && (to == 3)) {
      if (previous == 1) {
        turnRight();
        followNode(from, to);
      } else {
        drive(255, 255);
        delay(100);
        followNode(from, to);
      }
    } else if ((from == 6) && (to == 4)) {
      if (previous == 1) {
        turnLeft();
        followNode(from, to);
      } else {
        drive(255, 255);
        delay(100);
        followNode(from, to);
      }
    } else if ((from == 7) && (to == 2)) {
      if (previous == 1) {
        turnLeft();
        followNode(from, to);
      } else {
        drive(255, 255);
        delay(100);
        followNode(from, to);
      }
    } else if ((from == 7) && (to == 0)) {
      if (previous == 1) {
        turnRight();
        followNode(from, to);
      } else {
        drive(255, 255);
        delay(100);
        followNode(from, to);
      }
    } else {
      drive(255, 255);
      delay(100);
      followNode(from, to);
    }
  }


//------pathfinding-----



  struct Edge {
    uint8_t to;
    uint8_t w;
  };

  // Adjacency lists
  const Edge adj0[] = { {7,1}, {4,1} };
  const Edge adj1[] = { {7,1}, {6,1} };
  const Edge adj2[] = { {3,1}, {7,1} };
  const Edge adj3[] = { {6,2}, {2,1} };
  const Edge adj4[] = { {6,1}, {0,1} };
  const Edge adj5[] = { };
  const Edge adj6[] = { {3,2}, {4,2}, {1,1} };
  const Edge adj7[] = { {2,1}, {1,1}, {0,1} };

  // Graph table
  const Edge* graph[NODE_COUNT] = {
    adj0, adj1, adj2, adj3, adj4, adj5, adj6, adj7
  };

  const uint8_t deg[NODE_COUNT] = {2,2,2,2,2,0,3,3};
  // ------------------------------------------------------------
  // DIJKSTRA SHORTEST PATH
  // Given:
  //   start = starting node index
  //   goal  = target node index
  //   path  = output array that will store the node sequence
  //   maxLen = maximum size of 'path' array
  //
  // Returns:
  //   number of nodes in the path (>=1) if a path exists,
  //   0 if no path exists or if path buffer is too small.
  //
  // Path format:
  //   path[0] = start, path[len-1] = goal
  // ------------------------------------------------------------
  uint8_t dijkstraPath(uint8_t start, uint8_t goal,
                      uint8_t* path, uint8_t maxLen) {

    // A very large number used to represent "infinity"
    // (meaning: currently unknown / unreachable distance)
    const int INF = 32767;

    // dist[i] = best (smallest) known distance from start -> i
    int dist[NODE_COUNT];

    // prev[i] = previous node on the best path to i
    // This is used later to reconstruct the final path.
    // If prev[i] == -1, there is no predecessor known.
    int prev[NODE_COUNT];

    // used[i] = true when node i is finalized (we will not improve its distance)
    bool used[NODE_COUNT];

    // -------------------- INITIALIZATION --------------------
    // Set:
    //   - dist[] to INF (unknown)
    //   - prev[] to -1 (no predecessor)
    //   - used[] to false (nothing processed yet)
    for (int i = 0; i < NODE_COUNT; i++) {
      dist[i] = INF;
      prev[i] = -1;
      used[i] = false;
    }

    // Distance from start to itself is 0
    dist[start] = 0;

    // -------------------- MAIN DIJKSTRA LOOP --------------------
    // We run at most NODE_COUNT iterations. Each iteration:
    // 1) Choose the unused node with the smallest dist[]
    // 2) "Finalize" it (mark used)
    // 3) Relax its neighbors (try to improve paths through it)
    for (int i = 0; i < NODE_COUNT; i++) {

      // ----- Step 1: pick best unused node u -----
      // u will be the node that is:
      //   - not used yet
      //   - has the smallest dist[u]
      int u = -1;
      for (int j = 0; j < NODE_COUNT; j++) {
        if (!used[j] && (u == -1 || dist[j] < dist[u])) {
          u = j;
        }
      }

      // If u is still -1, there are no reachable unused nodes left.
      // That means goal might be unreachable.
      // Or: if u == goal, we can stop early (we already found the best path).
      if (u == -1 || u == goal) break;

      // ----- Step 2: finalize u -----
      // Once a node is selected as the smallest unused distance,
      // in Dijkstra this distance is final (cannot be improved later).
      used[u] = true;

      // ----- Step 3: relax all edges from u -----
      // "Relax" means: check if going from start -> u -> v is better
      // than the current best known path start -> v.
      for (int k = 0; k < deg[u]; k++) {

        int v = graph[u][k].to;
        int w = graph[u][k].w;

        // skip blocked edges
        if (blocked[u][v]) continue;

        if (dist[u] + w < dist[v]) {
          dist[v] = dist[u] + w;
          prev[v] = u;
        }           // record that best predecessor of v is u
      }
    }

    // -------------------- CHECK REACHABILITY --------------------
    // If goal still has INF distance, no path exists.
    if (dist[goal] == INF) return 0;

    // -------------------- PATH RECONSTRUCTION --------------------
    // We reconstruct the path by walking backward:
    //   goal -> prev[goal] -> prev[prev[goal]] ... until start
    //
    // This gives a reversed path, so we store into 'rev[]' first.
    uint8_t rev[16];   // small buffer (safe for NODE_COUNT=8)
    uint8_t len = 0;

    // Start from goal and repeatedly step to its predecessor
    for (int v = goal; v != -1; v = prev[v]) {
      rev[len++] = v;

      // (Optional safety) avoid overflow if something went wrong
      if (len >= sizeof(rev)) break;
    }

    // If path longer than caller's buffer, fail
    if (len > maxLen) return 0;

    // Now reverse it into the output 'path[]'
    // rev currently holds: goal, ..., start
    // output wants: start, ..., goal
    for (int i = 0; i < len; i++) {
      path[i] = rev[len - 1 - i];
    }

    // Return the number of nodes in the path
    return len;
 }
//----- drive path any node to node-----
  void drivePath(uint8_t start, uint8_t goal) {
while (start != goal) {

    obstacleFlag = false; // reset BEFORE planning

    uint8_t path[16];
    uint8_t len = dijkstraPath(start, goal, path, 16);
    if (len == 0) {
      Serial.println("No path found!");
      return;
    }

    // Follow edges along the path
    for (int i = 1; i < len; i++) {
      driveEdge(position, path[i]);   // followNode() updates position
      if (position == goal) return;

      if (obstacleFlag) {
        // we turned around, we are now back at 'position' (current)
        start = position; // re-plan from where we are now
        break;
      }
    }

    start = position;
  }
  }

//---obstacle detect-----


  float readDistanceCmOnce() {
    // Trigger pulse
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    // Measure echo pulse width (timeout ~25 ms ~ 4m)
    unsigned long us = pulseIn(ECHO, HIGH, 25000UL);
    if (us == 0) return NAN; // timeout / invalid

    // Speed of sound ~343 m/s => 29.1 us per cm round-trip ~ 58.2 us/cm
    float cm = us / 58.2f;

    if (cm < 0.5f || cm > MAX_VALID_CM) return NAN;
    return cm;
  }
  bool detectObstacle(){
    return (readDistanceCmOnce()< objThreshold);
  }
  




//-----------loop-------------- 
  void loop(){

    if  (routeLen<=0){
      drive(0,0);
      delay(1000);}
    else{
      for (int i=0; i<routeLen;i++){
        int dest = routeNodes[i];
        if (dest==position){continue;}
        Serial.print("Driving to : ");
        Serial.println(dest);
        drivePath(position,(uint8_t)dest);

        drive(0,0);
        delay(1000);
    
    }
    routeLen = 0;
    }
    }
// -------------------- SETUP --------------------
 void setup() {
  Serial.begin(9600);

  for (int i = 0; i < NODE_COUNT; i++) {
    for (int j = 0; j < NODE_COUNT; j++) {
      blocked[i][j] = false;
    }
  }


  pinMode(motor1PWM, OUTPUT);
  pinMode(motor1Phase, OUTPUT);
  pinMode(motor2PWM, OUTPUT);
  pinMode(motor2Phase, OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  analogReadResolution(12);        // 0..4095
  analogSetAttenuation(ADC_11db);  // best for ~0..3.3V

  analogWrite(motor1PWM, 0);
  analogWrite(motor2PWM, 0);

  lastTimeMs = millis();

  

  connectToWiFi();
  String routeStr = getRoute();
  if (routeStr.length() == 0) {
  Serial.println("No route received.");
  return;
 }

 if (!parseRouteDynamic(routeStr, routeNodes, routeLen)) {
  Serial.println("Failed to parse route!");
  return;
 }

  Serial.print("Route nodes: ");
  for (int i = 0; i < routeLen; i++) {
  Serial.print(routeNodes[i]);
  if (i < routeLen - 1) Serial.print(" -> ");
  }
  Serial.println();

  followNode(4,0);

    }