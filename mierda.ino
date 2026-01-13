/*
  TRIAC Delay
  This program sets up an interrupt handler for a rising edge digital input on Pin 19.
  The interrupt should delay for an appropriate period of time (assuming a 50 Hz AC
  input to the TRIAC circuit) to give the required duty-cycle phase control of the
  50Hz AC signal. After the delay, the program should assert Pin 18 low for an
  appropriate number of microseconds and then set it HIGH again.
*/
const int PIN_IN_ZC = 4;
const int PIN_OUT_TRIAC = 5;
volatile bool flag = false;

void setup() {  // put your setup code here, to run once:
//  Serial.begin(57600);
  // Disable Optocoupler
  pinMode(PIN_OUT_TRIAC, OUTPUT);
  digitalWrite(PIN_OUT_TRIAC, HIGH);  // The MOC3021M input is active-low

  // Set up a pin for the interrupt
  pinMode(PIN_IN_ZC,  INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_IN_ZC), readZC_ISR, RISING);
}

void loop() {
  if (flag) {
  //    Serial.println("In if statement");
    delayMicroseconds(1900);          // Delay to account for time between rising edge and zero-crossing
    delayMicroseconds(6000);          // Time per half cycle load is off 
    digitalWrite(PIN_OUT_TRIAC, LOW);  // Start the trigger pulse to the MOC3021M input
    delayMicroseconds(10);
    digitalWrite(PIN_OUT_TRIAC, HIGH); // End the trigger pulse to the MOC3021M input
    flag = false;
  }
}
void readZC_ISR() {  // ISR for Zero-crossing input
  flag = true; 
}
