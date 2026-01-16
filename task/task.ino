

//motor pins
int motor1PWM = 37;
int motor1Phase = 38;
int motor2PWM = 39;
int motor2Phase = 20;



// the setup routine runs once when you press reset:
void setup() {
Serial.begin(9600);
pinMode(motor1PWM, OUTPUT);
pinMode(motor1Phase, OUTPUT);
pinMode(motor2PWM, OUTPUT);
pinMode(motor2Phase, OUTPUT);
analogWrite(motor1PWM, 0);
analogWrite(motor2PWM, 0);
 }



// the loop routine runs over and over again continuously:
void loop() {
driveStraight(100);
delay(2000);
reverseStraight(100);
delay(2000);
   
}





// motor driving code
void rightFoward(int speed){
  digitalWrite(motor1Phase, HIGH); //forward
  analogWrite(motor1PWM, speed); // set speed of motor
  Serial.println("Right Forward"); // Display motor direction
}
void rightReverse(int speed){
  digitalWrite(motor1Phase, LOW); //reverse
  analogWrite(motor1PWM, speed); // set speed of motor
  Serial.println("Right Reverse"); // Display motor direction
}
void leftFoward(int speed){
  digitalWrite(motor2Phase, LOW); //forward
  analogWrite(motor2PWM, speed); // set speed of motor
  Serial.println("Left Forward"); // Display motor direction
  }
void leftReverse(int speed){
  digitalWrite(motor2Phase, HIGH); //reverse
  analogWrite(motor2PWM, speed); // set speed of motor
  Serial.println("Left Reverse"); // Display motor direction
  }

void driveStraight(int speed){
  rightFoward(speed);
  leftFoward(speed);
  }
void reverseStraight(int speed){
  rightReverse(speed);
  leftReverse(speed);
  }