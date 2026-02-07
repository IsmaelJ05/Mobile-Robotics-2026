int trig = 4;
int echo = 5;

long duration;
float distance;
float meter;

void setup()
{
  Serial.begin(9600);

  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  digitalWrite(trig, LOW);
  delay(2);

  Serial.println("Distance:");
}

void loop()
{
  // 🔹 Trigger HC-SR04 properly
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  // 🔹 Read echo pulse
  duration = pulseIn(echo, HIGH, 30000); // 30 ms timeout

  if (duration == 0)
  {
    Serial.println("Out of range");
  }
  else
  {
    distance = duration / 58.0;
    meter = distance / 100.0;

    Serial.print(distance);
    Serial.print(" cm\t");
    Serial.print(meter);
    Serial.println(" m");
  }

  delay(1000);
}
