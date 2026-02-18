/*
  ESP32-S3 sketch:
   - Reads 5 analog line sensors and converts them to 0/1
   - Reads ultrasonic distance and triggers outputs when close
   - Reads TWO RC-filtered PWM signals from another ESP (left/right motor command)
   - Estimates average speed from PWM and shows it on an SSD1306 OLED

  HARDWARE NOTES:
   - PWM sense lines must be RC-filtered (you said this is already working).
   - Share GND between the two ESPs.
   - DO NOT use GPIO3 for I2C on ESP32-S3 (strap pin -> upload problems).
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ================= OLED / I2C =================
#define OLED_ADDR 0x3C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// Choose safe I2C pins (NOT strap pins like GPIO3)
const int I2C_SDA = 12;
const int I2C_SCL = 11;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ================= PWM sense pins (RC-filtered) =================
// Must be ADC-capable pins on ESP32-S3 (GPIO 1..20)
const int PWM_L_SENSE = 9;   // left PWM -> RC -> this pin
const int PWM_R_SENSE = 10;   // right PWM -> RC -> this pin

// ADC settings
const int ADC_MAX = 4095;

// ---- Speed calibration (TUNE THESE!) ----
const int   PWM_DEAD = 60;      // below this, assume motor doesn't move
const float VMAX_MPS = 0.60f;   // max speed at PWM=255 in m/s (measure & set)

// ================= Line sensor pins =================
// Must be ADC-capable pins on ESP32-S3 (GPIO 1..20)
int AnalogPin[5] = {6, 7, 15, 16, 8};   // NOTE: if you use I2C SDA=8, don't also use 8 here!
int SensorDig[5] = {0, 0, 0, 0, 0};

int threshold = 800;
const bool SENSOR_HIGH_ON_LINE = true;

// ================= Ultrasonic + outputs =================
const int trig = 4;
const int echo = 5;
const int wall = 18;
const int obs  = 17;

long duration = 0;

// ================= Helpers: PWM -> speed =================
int readPwm255(int adcPin) {
  int raw = analogRead(adcPin);                          // 0..4095
  int pwm = (raw * 255 + (ADC_MAX / 2)) / ADC_MAX;       // 0..255
  if (pwm < 0) pwm = 0;
  if (pwm > 255) pwm = 255;
  return pwm;
}

float pwmToSpeedMps(int pwm) {
  if (pwm <= PWM_DEAD) return 0.0f;
  float u = (pwm - PWM_DEAD) / float(255 - PWM_DEAD);    // 0..1
  return u * VMAX_MPS;
}

float lowPass(float prev, float now, float alpha) {
  return prev + alpha * (now - prev);
}


// ================= Helpers: ultrasonic =================
float readDistanceCm() {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  duration = pulseIn(echo, HIGH, 30000); // 30ms timeout
  if (duration == 0) return 500.0f;

  return duration / 58.0f; // cm
}

// ================= OLED display (speed only) =================
void showOnOledSpeedOnly(float avgMps) {
display.clearDisplay();
display.setTextColor(SSD1306_WHITE);

// Title
display.setTextSize(2);
display.setCursor(0, 0);
display.print("Speed");

// Big number + unit on same line
display.setTextSize(3);
display.setCursor(0, 28);
display.print(avgMps * 3.6f, 1);
display.print(" kph");   // <-- same line, same size

display.display();

}

// ================= Setup / Loop =================
float speedFiltered = 0.0f;
void setup() {
  Serial.begin(9600);

  // --- pins ---
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(wall, OUTPUT);
  pinMode(obs, OUTPUT);

  digitalWrite(trig, LOW);
  digitalWrite(wall, LOW);
  digitalWrite(obs, LOW);

  // --- ADC ---
  analogReadResolution(12);
  pinMode(PWM_L_SENSE, INPUT);
  pinMode(PWM_R_SENSE, INPUT);
  for (int i = 0; i < 5; i++) pinMode(AnalogPin[i], INPUT);

  // --- I2C / OLED ---
  Wire.begin(I2C_SDA, I2C_SCL);     // <-- explicit pins (avoid GPIO3)
  Wire.setClock(100000);

  delay(200);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED init failed");
    while (1) delay(10);
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Ready");
  display.display();
  delay(500);
}

void loop() {
  // Obstacle logic
  float dist = readDistanceCm();
  if ((dist > 6) && (dist < 13)) {
    digitalWrite(wall, HIGH);
    delayMicroseconds(500);
    digitalWrite(wall, LOW);

    digitalWrite(obs, HIGH);
    delay(50);
    digitalWrite(obs, LOW);}

  // PWM reading
  int pwmL = readPwm255(PWM_L_SENSE);
  int pwmR = readPwm255(PWM_R_SENSE);

  // Speed estimate
  float vL = pwmToSpeedMps(pwmL);
  float vR = pwmToSpeedMps(pwmR);
  float vAvg = (vL + vR) * 0.5f;

  // Smooth for stable display
  speedFiltered = lowPass(speedFiltered, vAvg, 0.2f);

  // OLED
  showOnOledSpeedOnly(speedFiltered);
}
