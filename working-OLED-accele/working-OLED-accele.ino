#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>



//=================sensor read pins
int motor1pwm = 12;
int motor2pwm = 13;

int AnalogPin[5] = {6, 7, 15, 16, 8};
int SensorDig[5]= {0,0,0,0,0};

// Digital threshold
int threshold = 800;
const bool SENSOR_HIGH_ON_LINE = true;

//sensor reads
int sensorToDigital(int raw) {
  if (SENSOR_HIGH_ON_LINE) return (raw > threshold) ? 1 : 0;
  return (raw < threshold) ? 1 : 0;
}

void readSensor(){
  for (int i = 0; i < 5; i++) {
    SensorDig[i] = sensorToDigital(analogRead(AnalogPin[i]));
  }
}

//====== obs pins==============
int trig = 4;
int echo = 5;
int wall = 18;
int obs  = 17;

long duration;

// ========obs code===========
float readDistance() {
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

  if((dist > 7) && (dist < 9)){
    digitalWrite(wall, HIGH);
    delayMicroseconds(500);
    digitalWrite(wall, LOW);

    digitalWrite(obs, HIGH);
    delay(50);
    digitalWrite(obs, LOW);
  }
}

// ===== OLED =====
#define OLED_ADDR 0x3C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define I2C_SDA 10
#define I2C_SCL 3
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

unsigned long lastOledMs = 0;


//================== print to display
void showSpeedAndSensors(float mps) {
  display.clearDisplay();

  // ---- Speed ----
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Speed: ");
  display.print(mps * 3.6, 2);   // convert m/s to km/h
  display.println(" km/h");

  // ---- Sensor values ----
  display.setCursor(0, 20);
  display.print("S: ");

  for (int i = 0; i < 5; i++) {
    display.print(SensorDig[i]);
    display.print(" ");
  }

  display.display();
}
//=====setup=============
void setup() {
  Serial.begin(9600);

  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(wall, OUTPUT);
  pinMode(obs, OUTPUT);

  digitalWrite(wall, LOW);
  digitalWrite(obs, LOW);
  digitalWrite(trig, LOW);

  // If UNO/Nano: use Wire.begin(); (I2C pins are fixed)
  Wire.begin();
  Wire.setClock(100000);

  delay(2000);

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
  display.println("Ready");
  display.display();

  lastOledMs = millis();
}
 float speed = 10;
void loop() {
  testObs();
  showSpeedAndSensors(speed);
}
