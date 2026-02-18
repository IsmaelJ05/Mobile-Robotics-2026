#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
//====== obs pins==============
int trig = 4;
int echo = 5;
int wall = 18;
int obs = 17;

long duration;
float distance;
// ========obs code===========
float readDistance()
{
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  duration = pulseIn(echo, HIGH, 30000);

  if (duration == 0) return 500;

  return duration / 58.0;
}
 void testObs(){
    float dist = readDistance();

 if((dist>5)&&(dist<7)){

  digitalWrite(wall, LOW);
  digitalWrite(wall, HIGH);
  delayMicroseconds(500);
  digitalWrite(wall, LOW);
  digitalWrite(obs, LOW);
  digitalWrite(obs, HIGH);
  delay(50);
  digitalWrite(obs, LOW);
  


 }

 }

// ===== I2C pins & OLED address =====
#define I2C_SDA   3
#define I2C_SCL   10
#define OLED_ADDR 0x3C   // change to 0x3D if needed

// ===== OLED =====
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ===== MMA8451 =====
Adafruit_MMA8451 mma = Adafruit_MMA8451();

// ===== Timing =====
const uint32_t IMU_PERIOD_MS  = 10;    // 100 Hz accel update
const uint32_t OLED_PERIOD_MS = 2000;  // display refresh every 2 seconds

unsigned long lastImuMs  = 0;
unsigned long lastOledMs = 0;

// ===== Speed estimate (X axis only) =====
float vx = 0.0f;        // m/s (estimated)
float ax_bias = 0.0f;   // m/s^2 bias from calibration

// ===== Tuning =====
const float ACCEL_DEADBAND = 0.16f;   // ignore tiny noise (m/s^2)
const float VELOCITY_DECAY = 0.92f;   // drift reduction when accel ~0
const float STOP_ACCEL_THRESH = 0.28f;

void calibrateX(unsigned samples = 300) {
  sensors_event_t e;
  float sum = 0.0f;

  for (unsigned i = 0; i < samples; i++) {
    mma.getEvent(&e);
    sum += e.acceleration.x;
    delay(5);
  }
  ax_bias = sum / samples;
}

void showSpeed(float mps) {
  //if (mps>1.0) return;
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 20);
  display.print(mps*3.6, 2);
  display.print(" km/h");
  display.display();
}

void setup() {
  Serial.begin(9600);

  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(wall, OUTPUT);
  pinMode(obs, OUTPUT);

  digitalWrite(wall, LOW);
  digitalWrite(obs, LOW);
  digitalWrite(trig, LOW);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  // Let I2C devices power up fully (helps avoid startup NACK)
  delay(2000);

  // OLED init (retry a few times)
  bool oled_ok = false;
  for (int i = 0; i < 5 && !oled_ok; i++) {
    oled_ok = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
    if (!oled_ok) delay(200);
  }
  if (!oled_ok) {
    Serial.println("OLED init failed");
    while (1) delay(10);
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("OLED OK");
  display.display();

  // MMA init (try both common addresses, retry)
  bool mma_ok = false;
  for (int i = 0; i < 5 && !mma_ok; i++) {
    mma_ok = mma.begin(0x1D) || mma.begin(0x1C);
    if (!mma_ok) delay(200);
  }
  if (!mma_ok) {
    Serial.println("MMA8451 init failed");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("MMA FAIL");
    display.display();
    while (1) delay(10);
  }

  mma.setRange(MMA8451_RANGE_2_G);

  // Calibrate bias on X while still
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Keep still...");
  display.println("Calibrating X");
  display.display();

  calibrateX();

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Ready");
  display.display();

  lastImuMs  = millis();
  lastOledMs = millis();
}

void loop() {
  testObs();

  unsigned long now = millis();

  // ===== IMU update @ 100 Hz =====
  if (now - lastImuMs >= IMU_PERIOD_MS) {
    float dt = (now - lastImuMs) / 1000.0f;
    lastImuMs = now;

    sensors_event_t e;
    mma.getEvent(&e);

    // X axis only
    float ax = e.acceleration.x - ax_bias;

    // Noise removal
    if (fabs(ax) < ACCEL_DEADBAND) ax = 0.0f;

    static float ax_filtered = 0;
    ax_filtered = 0.85f * ax_filtered + 0.15f * ax;
    vx += ax_filtered * dt;

    // Integrate accel -> velocity
    if (fabs(ax) > 0.25f) {
    vx += ax * dt;
}

    // Consider stationary if accel near 0 for a while
    static int stillCount = 0;
    if (fabs(ax) < 0.12f) stillCount++;
    else stillCount = 0;

    if (stillCount > 30) {   // ~300 ms at 100 Hz
      vx = 0.0f;
    }

  // ===== OLED update every 2 seconds =====
  if (now - lastOledMs >= OLED_PERIOD_MS) {
    lastOledMs = now;
    showSpeed(fabs(vx));
  }
}
}
