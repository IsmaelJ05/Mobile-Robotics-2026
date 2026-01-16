/*
  DIGITAL PID Line Follower (ESP32-S3) — FULL CODE (with stability + corner authority fixes)
  ---------------------------------------------------------------------------------------
  Implemented changes (requested):
    ✅ Gain scheduling (soft near center, strong in corners)
    ✅ Variable maxTurn (low on straights, higher in corners)
    ✅ Increased corner slow-down
    ✅ Turn slew-rate limit (kills sharp snapping oscillation)
    ✅ Derivative filter alpha adjusted for digital error steps

  IMPORTANT sanity:
    - Your stated ranges: off-line ~300–600, on-line ~2000–4095
      => RAW is HIGH when on the line, so SENSOR_HIGH_ON_LINE should be true.
*/

#include <Arduino.h>

// -------------------- TUNING (START VALUES) --------------------
int delaySet  = 0;
int baseSpeed = 175;          // start lower while tuning

// Keep Ki = 0 for digital sensors until everything else is stable
float Ki = 0.0;

// Anti-windup (mostly irrelevant with Ki=0, but kept for later)
float integralLimit = 200.0f;

// Weights for 5 sensors
int weights[5] = {-2, -1, 0, 1, 2};

// Digital threshold
int threshold = 1200;                 // adjust 1100–1600 if needed
const bool SENSOR_HIGH_ON_LINE = true; // <-- FIXED for your sensor ranges

// Corner slow-down (increased)
int slowDown1 = 20;          // when abs(error)==1
int slowDown2 = 90;          // when abs(error)>=2

// Gain scheduling (soft in center, stronger in corners)
float Kp_center = 18.0f;     // absErr <= 1
float Kd_center = 18.0f;

float Kp_corner = 32.0f;     // absErr >= 2
float Kd_corner = 26.0f;

// Turn clamp scheduling
float maxTurn_center = 80.0f;
float maxTurn_corner = 200.0f;

// Derivative smoothing (less laggy than 0.95 for digital steps)
const float dAlpha_use = 0.80f;

// Turn slew-rate limit (prevents sharp snapping)
float turnSlewRate = 250.0f; // "turn units per second" (tune 150..400)

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
float lastError = 0.0f;
float dFilt = 0.0f;
float lastTurn = 0.0f;
unsigned long lastTimeMs = 0;

// -------------------- UTILS --------------------
int clamp255(int v) {
  if (v < 0) return 0;
  if (v > 255) return 255;
  return v;
}

int sensorToDigital(int raw) {
  // Returns 1 if sensor "sees the line", else 0
  if (SENSOR_HIGH_ON_LINE) return (raw > threshold) ? 1 : 0;
  return (raw < threshold) ? 1 : 0;
}

// -------------------- MOTOR CONTROL --------------------
void rightFoward(int speed){
  digitalWrite(motor1Phase, HIGH);
  analogWrite(motor1PWM, speed);
}
void rightReverse(int speed){
  digitalWrite(motor1Phase, LOW);
  analogWrite(motor1PWM, speed);
}
void leftFoward(int speed){
  digitalWrite(motor2Phase, LOW);
  analogWrite(motor2PWM, speed);
}
void leftReverse(int speed){
  digitalWrite(motor2Phase, HIGH);
  analogWrite(motor2PWM, speed);
}

// Forward-only (stable for PID)
void driveForwardSpeeds(int rightSpeed, int leftSpeed) {
  rightFoward(clamp255(rightSpeed));
  leftFoward(clamp255(leftSpeed));
}

// -------------------- READ DIGITAL ERROR --------------------
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

  // Keep lastError when lost so recovery spins the correct way
  if (lineLostOut) return lastError;

  return (float)e;
}

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(9600);

  pinMode(motor1PWM, OUTPUT);
  pinMode(motor1Phase, OUTPUT);
  pinMode(motor2PWM, OUTPUT);
  pinMode(motor2Phase, OUTPUT);

  // ESP32-S3 ADC setup (makes analogRead consistent)
  analogReadResolution(12);        // 0..4095
  analogSetAttenuation(ADC_11db);  // best for ~0..3.3V

  analogWrite(motor1PWM, 0);
  analogWrite(motor2PWM, 0);

  lastTimeMs = millis();
}

// -------------------- LOOP --------------------
void loop() {
  delay(delaySet);

  // 1) Read digital line error
  bool lineLost = false;
  float error = readLineErrorDigital(lineLost);

  // (Removed the "center sensor + error +/-1 => 0" dead-zone; it can cause snap/limit cycles)

  // 2) Compute dt
  unsigned long now = millis();
  float dt = (now - lastTimeMs) / 1000.0f;
  if (dt <= 0.005f) dt = 0.005f;
  lastTimeMs = now;

  // 3) Line lost recovery: arc-spin based on lastError sign (forward-only)
  if (lineLost) {
    int spin = 60; // tune 40–90
    if (lastError >= 0) driveForwardSpeeds(baseSpeed - spin, baseSpeed + spin);
    else                driveForwardSpeeds(baseSpeed + spin, baseSpeed - spin);

    // Debug
    Serial.print("LOST raw:");
    for (int i = 0; i < N; i++) {
      Serial.print(AnalogValue[i]);
      Serial.print(i == N-1 ? " " : ",");
    }
    Serial.print(" dig:");
    for (int i = 0; i < N; i++) Serial.print(DigitalValue[i]);
    Serial.println();
    return;
  }

  // 4) Corner slow-down
  int absErr = abs((int)error);
  int localBase = baseSpeed;
  if (absErr >= 2) localBase = baseSpeed - slowDown2;
  else if (absErr == 1) localBase = baseSpeed - slowDown1;

  // 5) PID core (with gain scheduling)
  integral += error * dt;
  if (integral >  integralLimit) integral =  integralLimit;
  if (integral < -integralLimit) integral = -integralLimit;

  float dRaw = (error - lastError) / dt;
  lastError = error;

  // Derivative smoothing
  dFilt = dAlpha_use * dFilt + (1.0f - dAlpha_use) * dRaw;

  // Gain scheduling: soft near center, stronger in corners
  float Kp_use = (absErr >= 2) ? Kp_corner : Kp_center;
  float Kd_use = (absErr >= 2) ? Kd_corner : Kd_center;

  float turn = (Kp_use * error) + (Ki * integral) + (Kd_use * dFilt);

  // 6) Variable maxTurn: smaller on straights, larger in corners
  float maxTurn_use = (absErr >= 2) ? maxTurn_corner : maxTurn_center;
  if (turn >  maxTurn_use) turn =  maxTurn_use;
  if (turn < -maxTurn_use) turn = -maxTurn_use;

  // If your robot turns the wrong way AFTER fixing SENSOR_HIGH_ON_LINE,
  // fix motor direction wiring/logic first. Only as a last resort, flip here:
  // turn = -turn;

  // 7) Turn slew-rate limiting (prevents sharp oscillation/snapping)
  float maxTurnStep = turnSlewRate * dt;
  float delta = turn - lastTurn;
  if (delta >  maxTurnStep) turn = lastTurn + maxTurnStep;
  if (delta < -maxTurnStep) turn = lastTurn - maxTurnStep;
  lastTurn = turn;

  // Optional tiny deadband on turn to stop micro-jitter around zero:
  // if (fabs(turn) < 3.0f) turn = 0.0f;

  // 8) Convert to motor speeds
  int rightSpeed = (int)(localBase - turn);
  int leftSpeed  = (int)(localBase + turn);

  // 9) Drive
  driveForwardSpeeds(rightSpeed, leftSpeed);
}
