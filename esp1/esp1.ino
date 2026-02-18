#include <WiFi.h>
#include <HTTPClient.h>
#include <Arduino.h>
#include <ESP32Servo.h>
//--------------PID profiles---------


  // Forward declare (helps Arduino's auto-prototype system)
  struct PIDProfile;

  // The profile struct
  struct PIDProfile {
    int baseSpeed;
    int slowDown1;
    int slowDown2;
    float Kp_center;
    float Kd_center;
    float Kp_corner;
    float Kd_corner;
    float maxTurn_center;
    float maxTurn_corner;
    float innerErrorScale;
    float innerDScale;
    float dAlpha;
    float turnSlewRate;
  };

  // --- PROFILES ---

  // Profile for normal edges
  PIDProfile pid_w1 = {
    /*baseSpeed*/ 250,
    /*slowDown1*/ 70,
    /*slowDown2*/ 120,
    /*Kp_center*/ 28.0f,
    /*Kd_center*/ 12.0f,
    /*Kp_corner*/ 40.0f,
    /*Kd_corner*/ 24.0f,
    /*maxTurn_center*/ 95.0f,
    /*maxTurn_corner*/ 250.0f,
    /*innerErrorScale*/ 0.60f,
    /*innerDScale*/ 0.60f,
    /*dAlpha*/ 0.88f,
    /*turnSlewRate*/ 200
  };

  // Profile for slower / heavy edges
  PIDProfile pid_w2 = {
    /*baseSpeed*/ 250,
    /*slowDown1*/ 100,
    /*slowDown2*/ 170,
    /*Kp_center*/ 28.0f,
    /*Kd_center*/ 12.0f,
    /*Kp_corner*/ 40.0f,
    /*Kd_corner*/ 24.0f,
    /*maxTurn_center*/ 150.0f,
    /*maxTurn_corner*/ 300.0f,
    /*innerErrorScale*/ 0.50f,
    /*innerDScale*/ 0.70f,
    /*dAlpha*/ 0.90f,
    /*turnSlewRate*/ 400.0f
  };

  // --- ACTIVE (used by follow()) ---
  int baseSpeed = 220;
  int slowDown1 = 40;
  int slowDown2 = 150;

  float Kp_center = 28.0f;
  float Kd_center = 12.0f;
  float Kp_corner = 40.0f;
  float Kd_corner = 24.0f;

  float maxTurn_center = 95.0f;
  float maxTurn_corner = 350.0f;

  float innerErrorScale = 0.60f;
  float innerDScale = 0.60f;

  float dAlpha_use = 0.88f;
  float turnSlewRate = 350.0f;

  // These were referenced in your follow() code
  float Ki = 0.0f;
  float integralLimit = 200.0f;

  // Helper: copy a profile into the active variables
  void applyPIDProfile(const PIDProfile& p) {
    baseSpeed = p.baseSpeed;
    slowDown1 = p.slowDown1;
    slowDown2 = p.slowDown2;

    Kp_center = p.Kp_center;
    Kd_center = p.Kd_center;
    Kp_corner = p.Kp_corner;
    Kd_corner = p.Kd_corner;

    maxTurn_center = p.maxTurn_center;
    maxTurn_corner = p.maxTurn_corner;

    innerErrorScale = p.innerErrorScale;
    innerDScale = p.innerDScale;

    dAlpha_use = p.dAlpha;
    turnSlewRate = p.turnSlewRate;
  }

// ===== Wi-Fi details =====
  const char* ssid     = "iot";
  const char* password = "premorbidly55telephoning";

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

//------pathfinding-----

  enum Node { N0, N1, N2, N3, N4, N5, N6, N7, NODE_COUNT };
  bool blocked[NODE_COUNT][NODE_COUNT] = {};
  int blockedFrom = -1;
  int blockedTo   = -1;

  struct Edge {
    uint8_t to;
    uint8_t w;
  };

  // Adjacency lists
  const Edge adj0[] = { {7,2}, {4,2} };
  const Edge adj1[] = { {7,2}, {6,1} };
  const Edge adj2[] = { {3,2}, {7,2} };
  const Edge adj3[] = { {6,5}, {2,2} };
  const Edge adj4[] = { {6,5}, {0,2} };
  const Edge adj5[] = {6,10 };
  const Edge adj6[] = { {3,5}, {4,5}, {1,1}, {5,10} };
  const Edge adj7[] = { {2,2}, {1,2}, {0,2} };

  // Graph table
  const Edge* graph[NODE_COUNT] = {
    adj0, adj1, adj2, adj3, adj4, adj5, adj6, adj7
  };

  const uint8_t deg[NODE_COUNT] = {2,2,2,2,2,1,4,3};

  int getEdgeWeight(int from, int to) {
  for (int k = 0; k < deg[from]; k++) {
    if (graph[from][k].to == to) return graph[from][k].w;
  }
  return 1; // default if not found
  }


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

        // v = neighbor node
        int v = graph[u][k].to;

        // w = edge weight cost from u to v
        int w = graph[u][k].w;
        if (blocked[u][v]) continue;
        // If dist[u] is already INF, u was unreachable (shouldn't happen here)
        // Check if going via u improves dist[v]:
        //   dist[u] + w < dist[v]
        if (dist[u] + w < dist[v]) {
          dist[v] = dist[u] + w; // update best known distance to v
          prev[v] = u;           // record that best predecessor of v is u
        }
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

// -------------------- TUNING (START VALUES) ------------------
  int nodeCrossDelay= 75;
  
  int delaySet  = 0;
  

  // Weights for 5 sensors
  int weights[5] = {2, 1, 0, -1, -2};

  // Digital threshold
  int threshold = 800;                  // adjust 1100–1600 if needed
  const bool SENSOR_HIGH_ON_LINE = true; // RAW HIGH when on line (your sensor ranges)

  

// -------------------- PINS --------------------
  int motor1PWM   = 37;  // Right motor PWM
  int motor1Phase = 38;  // Right motor direction
  int motor2PWM   = 39;  // Left motor PWM
  int motor2Phase = 20;  // Left motor direction
  
  int obsInterrupt = 17;
  int wallInterrupt = 19; 

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
//========servo control==========
 Servo s;
  const int SERVO_PIN = 18;
  const int PULSE_MIN = 500;
  const int PULSE_MAX = 2500;
  const int PULSE_STOP = 1500;  // calibrate later if needed



  int clampi(int v, int lo, int hi) {
    return v < lo ? lo : (v > hi ? hi : v);
  }


  int servoFollow(float turn, float maxTurn_use) {
    // ---- TUNING KNOBS ----
    static int stopPulseUs = 1500;  // CALIBRATE: true "stop" (e.g. 1492..1510)
    static int spanUs = 800;        // how strong steering is (start 150..300)
    static float deadband = 0.00f;  // ignore tiny commands near center (0.02..0.10)
    static float servoScale = 1.5f;
    static int pulseMinUs = 500;  // clamp range (keep conservative at first)
    static int pulseMaxUs = 2500;

    static float rateLimitUsPerSec = 00.0f;  // smoothness (800..3000). 0 disables.
    // ----------------------

    static int lastUs = 1500;
    static unsigned long lastMs = 0;

    // Safety
    if (maxTurn_use < 1.0f) return stopPulseUs;

    // 1) Normalize to -1..+1
    float x = (turn / maxTurn_use) ;
    if (x > 1.0f) x = 1.0f;
    if (x < -1.0f) x = -1.0f;

    x*= servoScale;

    // 2) Deadband around neutral to prevent creep/jitter
    if (fabsf(x) < deadband) x = 0.0f;

    // 3) Map to microseconds around stop
    int targetUs = stopPulseUs + (int)(x * (float)spanUs);

    // 4) Clamp to safe range
    if (targetUs < pulseMinUs) targetUs = pulseMinUs;
    if (targetUs > pulseMaxUs) targetUs = pulseMaxUs;

    // 5) Optional rate limiting (prevents servo command snapping)
    unsigned long now = millis();
    if (lastMs == 0) lastMs = now;
    float dt = (now - lastMs) / 1000.0f;
    if (dt < 0.001f) dt = 0.001f;
    lastMs = now;

    if (rateLimitUsPerSec > 0.0f) {
      int maxStep = (int)(rateLimitUsPerSec * dt);
      int delta = targetUs - lastUs;
      if (delta > maxStep) targetUs = lastUs + maxStep;
      if (delta < -maxStep) targetUs = lastUs - maxStep;
    }

    lastUs = targetUs;
    return targetUs;
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

//----- detector interupts---------
  volatile bool goWall = false;
  volatile bool parked = false;
  volatile bool obsFlag = false;
  volatile bool reroute = false;
  volatile int curFrom = -1;
  volatile int curTo   = -1;
  volatile bool wallArmed = false;
  portMUX_TYPE isrMux = portMUX_INITIALIZER_UNLOCKED;
  volatile uint32_t wallArmMs = 0;

  void IRAM_ATTR wall() {
    if (!goWall) return;
    parked = true;
    Serial.println("int");
    return;

  }
  void gotoWall() {
    attachInterrupt(digitalPinToInterrupt(wallInterrupt), wall, RISING);
    // Arm after a short delay to avoid the attach-edge / noise
    wallArmMs = millis();
    while (millis() - wallArmMs < 150) {    // 100–300ms works
      drive(0,0);
      delay(1);
    }
    
    drive(150, 153);//156
  //to remove obstacal sequence comment from here

    while (true){
      testObstacle();
      if (reroute){
        reroute=false;
        break;}
    }
    drive(-200,200);
    delay(350);
    drive(0,0);
    delay(50);
    drive(100,101);//103
    delay(1500);
    drive(0,0);
    delay(50);
    drive(200,-200);
    delay(350);
    drive(0,0);
  //====================
    
    goWall = true;
    parked= false;

    while (!parked) {
      drive(100, 101);//103
      delay(10);
    }

    drive(0,0);
    detachInterrupt(digitalPinToInterrupt(wallInterrupt));
    drive(35,35);
    delay(200);
    drive(0,0);
    Serial.println("parked");
    }

  void testObstacle(){
    if(digitalRead(obsInterrupt)==0){return;}
    drive(0,0);
    delay(500);
    int from, to;
    portENTER_CRITICAL(&isrMux);
    from = curFrom;
    to   = curTo;
    portEXIT_CRITICAL(&isrMux);

    if (from >= 0 && to >= 0 && from < NODE_COUNT && to < NODE_COUNT) {
      blocked[from][to] = true;
      blocked[to][from] = true;
      blockedFrom = from;
      blockedTo = to;

      Serial.print("Blocked edge: ");
      Serial.print(from);
      Serial.print(" <-> ");
      Serial.println(to);
      }
      reroute= true;
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
    /*if (lineLost) {
      // Quick spin-search to reacquire (stronger than arcing)
      int search = 120;            // tune 90..160
      int base   = baseSpeed - 40; // slow a bit while searching

      if (lastEUse >= 0) drive(base - search, base + search);
      else               drive(base + search, base - search);

      // Don’t run PID this cycle
      return;
    }*/

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


    int steerUs = servoFollow(turn, maxTurn_use);
    s.writeMicroseconds(steerUs);

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
      drive(230,230);
      delay(130);
      drive(0,0);
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
        drive(230,230);
        delay(150);
        drive(0,0);
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
        portENTER_CRITICAL(&isrMux);
        curFrom = from;
        curTo   = to;
        portEXIT_CRITICAL(&isrMux);
        
        if (previous==to){turn180();}
        obsFlag = false;
        while (true){
              testObstacle();
              if (reroute){
              turn180();
              while(true){
                follow();
                if (detectNode()){
                  drive(0,0);
                  previous = to;
                  position = from;
                  return;
                    }
                  }
              	}
              follow();
              if (detectNode()){
                drive(0,0);
                previous = from;
                position = to;
                break;
                }
      }
      }

 void driveEdge(int from, int to) {
  int w = getEdgeWeight(from, to);

  // choose speed based on weight
  if (w > 3) {
      applyPIDProfile(pid_w2);
    } else {
      applyPIDProfile(pid_w1);
    }
  
  portENTER_CRITICAL(&isrMux);
  curFrom = from;
  curTo   = to;
  portEXIT_CRITICAL(&isrMux);
    if ((from==6) && (to==5)){
      if (previous == 4){turnLeft();}
      if (previous == 3){turnRight();}
      if (previous == 1){drive(255,255);
        delay(nodeCrossDelay);
      }
      for (int i=0; i<30; i++){
      delay(10);
      follow();
      }
      drive (220,220);
      gotoWall();
      position=5;
      return;

    }
    if ((from == 6) && (to == 1)) {
      if (previous == 4) {
        turnRight();
        followNode(from, to);
      } else if (previous == 3) {
        turnLeft();
        followNode(from, to);
      } else {
        drive(255, 255);
        delay(nodeCrossDelay);
        followNode(from, to);
      }
      return;
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
        delay(nodeCrossDelay);
        followNode(from, to);
      }
      return;
    }

    else if ((from == 6) && (to == 3)) {
      if (previous == 1) {
        turnRight();
        followNode(from, to);
      } else {
        drive(255, 255);
        delay(nodeCrossDelay);
        followNode(from, to);
      }
      return;
    } 
    else if ((from == 6) && (to == 4)) {
      if (previous == 1) {
        turnLeft();
        followNode(from, to);
      } else {
        drive(255, 255);
        delay(nodeCrossDelay);
        followNode(from, to);
      }
    return;
    }
    
     else if ((from == 7) && (to == 2)) {
      if (previous == 1) {
        turnLeft();
        followNode(from, to);
      } else {
        drive(255, 255);
        delay(nodeCrossDelay);
        followNode(from, to);
      }
    return;
    } 
    else if ((from == 7) && (to == 0)) {
      if (previous == 1) {
        turnRight();
        followNode(from, to);
      } else {
        drive(255, 255);
        delay(nodeCrossDelay);
        followNode(from, to);
      }
    return;
    } 
    else {
      drive(255,255);
      delay(nodeCrossDelay);
      followNode(from, to);
      return;
    }
  }



//----- drive path any node to node-----
  void drivePath( uint8_t goal) {
    while (position!= goal){
      uint8_t path[16];
      uint8_t len = dijkstraPath(position, goal, path, 16);
      reroute= false;
      if (len == 0) {
        Serial.println("No path found!");
        return;
      }
      for(int i=1; i<len; i++){
        driveEdge(position,path[i]);
        if (reroute) {
          break;
        }
      
      }

    }
    drive(0,0);
    sendArrival(position);
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
        drivePath((uint8_t)dest);
        drive(0,0);
        delay(100);
      }

    delay(1000);
    routeLen = 0;
    }
    }
    
// -------------------- SETUP --------------------
 void setup() {
  Serial.begin(9600);



  pinMode(motor1PWM, OUTPUT);
  pinMode(motor1Phase, OUTPUT);
  pinMode(motor2PWM, OUTPUT);
  pinMode(motor2Phase, OUTPUT);
  drive(0,0);
  delay(500);
  drive(0,0);
  delay(500);
  drive(0,0);
  delay(500);
  drive(0,0);

  s.setPeriodHertz(50);                  // standard servo PWM
  s.attach(SERVO_PIN, PULSE_MIN, PULSE_MAX);
  s.writeMicroseconds(1500);  

  pinMode(obsInterrupt, INPUT_PULLDOWN);
  pinMode(wallInterrupt, INPUT_PULLDOWN);
  //attachInterrupt(digitalPinToInterrupt(obsInterrupt),obstacle,RISING);

  analogReadResolution(12);        // 0..4095
  analogSetAttenuation(ADC_11db);  // best for ~0..3.3V


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


  position = 4;
  driveEdge(4,0);
  drive(0,0);
  sendArrival(position);
  delay(200);
    }

//==end==