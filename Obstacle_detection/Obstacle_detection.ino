int trig = 4;
int echo = 5;
int wall = 7;
int obs = 6;
long duration;
float distance;

void setup()
{
  Serial.begin(9600);

  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(wall, OUTPUT);
  pinMode(obs, OUTPUT);

  digitalWrite(wall, LOW);
  digitalWrite(obs, LOW);

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
    distance = 500;
  }
  else
  {
    distance = duration / 58.0;

  }
  if (distance < 10.0 ){
    digitalWrite(wall,HIGH);
    delayMicroseconds(500);
    Serial.println("pulse");
    
  }

  

  delay(100);

}
