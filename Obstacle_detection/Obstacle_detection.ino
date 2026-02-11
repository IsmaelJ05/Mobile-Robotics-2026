int trig = 4;
int echo = 5;
int wall = 18;

long duration;
float distance;


void setup()
{
  Serial.begin(9600);

  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(wall, OUTPUT);

  digitalWrite(wall, LOW);
  digitalWrite(trig, LOW);
}

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

void loop()
{
  float dist = readDistance();
 if((dist>2.0)&&(dist<3.0)){
  digitalWrite(wall, LOW);
  digitalWrite(wall, HIGH);
  delayMicroseconds(500);
  digitalWrite(wall, LOW);
  Serial.println(dist);
 }
  
}