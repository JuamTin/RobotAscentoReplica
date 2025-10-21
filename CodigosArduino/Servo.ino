//*****************************************************   PWM ServoMotor    *****************************************************//
const int modulePWMPin = 0;  // the PWM pin
const int freq = 50;         // set the frequency for 50Hz -> T = 20ms
const int pwmChannel = 4;    // set the PWM channel
const int resolution = 8;    // set PWM resolution
int setAngle = 270;             // Set Desired angle to 360*******************
float PWM = 0;
float dutyCycle = 0;
const long intervalPWM = 15;          // desire milisecond interval
unsigned long previousMillisPWM = 0;  // variable to store the previous timestamp
//*****************************************************************************************************************************
void setup() {
  //Servomotor
  ledcSetup(pwmChannel, freq, resolution);  // define the PWM Setup
  ledcAttachPin(modulePWMPin, pwmChannel);
  Serial.begin(9600);  // open a serial connection to your computer
}

void loop() {
  unsigned long currentMillisPWM = millis();  // Actual time variable PWM
  if (currentMillisPWM - previousMillisPWM >= intervalPWM) {
    previousMillisPWM = currentMillisPWM;
    PWM = calculatingPWM(setAngle);
    dutyCycle = mapDutyCycle(PWM);
    ledcWrite(pwmChannel, dutyCycle);
  }
}
//ServoMotor
float calculatingPWM(int valueAngle) {
  return (valueAngle - 135) / 0.09;
}
int mapDutyCycle(float valuePWM) {
  float duty = (valuePWM / (valuePWM + ((1 / freq) - valuePWM))) * 100;  // DutyC = 100 * Ton/(Ton + Toff)
  return map(duty, 0, 100, 0, 255);
}