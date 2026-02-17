#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ===== I2C pins & OLED address =====
#define I2C_SDA   8
#define I2C_SCL   9
#define OLED_ADDR 0x3C   // change to 0x3D if needed

// ===== OLED =====
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ===== MPU6050 =====
Adafruit_MPU6050 mpu;

// ===== Speed estimate (Y axis only) =====
float v_forward = 0.0f;     // forward speed (m/s)
float ay_offset = 0.0f;     // stationary Y-axis bias
bool calibrated = false;

unsigned long lastImuMs  = 0;
unsigned long lastOledMs = 0;

const uint32_t IMU_PERIOD_MS  = 10;     // 100 Hz
const uint32_t OLED_PERIOD_MS = 1000;   // update screen every 1 second

const float ACCEL_DEADBAND = 0.12f;     // ignore small noise
const float VELOCITY_DECAY = 0.90f;     // reduce drift when stopped

// ============================================================
// Calibrate Y axis while robot is stationary
// ============================================================
void calibrateIMU(unsigned samples = 200) {
  sensors_event_t a, g, t;
  float sum = 0.0f;

  for (unsigned i = 0; i < samples; i++) {
    mpu.getEvent(&a, &g, &t);
    sum += a.acceleration.y;   // Y axis only
    delay(5);
  }

  ay_offset = sum / samples;
  calibrated = true;
}

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  delay(100);

  // OLED init
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED init failed");
    while (1);
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Calibrating...");
  display.display();

  // MPU init
  if (!mpu.begin()) {
    Serial.println("MPU6050 init failed");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Calibrate while still
  calibrateIMU();

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Ready");
  display.display();

  lastImuMs  = millis();
  lastOledMs = millis();
}

// ============================================================
// Loop
// ============================================================
void loop() {
  unsigned long now = millis();

  // ---- IMU update (100 Hz) ----
  if (now - lastImuMs >= IMU_PERIOD_MS) {
    float dt = (now - lastImuMs) / 1000.0f;
    lastImuMs = now;

    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);

    // Use ONLY Y axis
    float ay = a.acceleration.y - (calibrated ? ay_offset : 0.0f);

    // Remove tiny noise
    if (fabs(ay) < ACCEL_DEADBAND)
      ay = 0.0f;

    // Integrate acceleration -> velocity
    v_forward += ay * dt;

    // Reduce drift when not accelerating
    if (ay == 0.0f) {
      v_forward *= VELOCITY_DECAY;
      if (fabs(v_forward) < 0.01f)
        v_forward = 0.0f;
    }
  }

  // ---- OLED update (1 Hz) ----
  if (now - lastOledMs >= OLED_PERIOD_MS) {
    lastOledMs = now;

    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 20);
    display.print(fabs(v_forward), 2);
    display.print(" m/s");
    display.display();
  }
}
