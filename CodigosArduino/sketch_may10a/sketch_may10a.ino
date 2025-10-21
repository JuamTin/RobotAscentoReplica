#include <Arduino.h>
#include "motor.h"
#include "motor2.h"
#include "robot_position.h"
#include "control_trajectory.h"
#include "util.h"
#include "servo.h"
#include "imu_manager.h"

const float R = 0.075;
const float L = 0.24;
const float wMaxR = 11.0;
const float wMaxL = 11.0;
// Servos
const int PWMPinAzul = 27;
const int PWMPinRojo = 12;
const int freq = 488;
const int ChannelServoAzul = 4;
const int ChannelServoRojo = 5;
const int resolution = 8;

const float xTarget = 1.5;
const float yTarget = 0;

// interruptions
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


servo servoAzul(90, ChannelServoAzul);
servo servoRojo(90, ChannelServoRojo);

ImuManager imuManager;

RobotPosition robotPosition = RobotPosition(R, L);
ControlTrajectory controlTrajectory = ControlTrajectory(R, L, wMaxR, wMaxL);

// const int N= 1000;
// float angR[N];
// float angL[N];
// float wL[N];
// float wR[N];



Motor motoRigth = Motor(
  32,      // pinPulseA
  33,      // pinPulseB
  2,       // modulePWMPin
  5,       // modulePWMPinNeg
  0,       // pwmChannel0
  1,       // pwmChannel1
  6505,    // R,
  15.99,   // u0
  -6.576,  // u1
  -7.715,  // u2
  0.9847,  // y0
  0.01531  // y1
);


Motor2 motoLeft = Motor2(
  25,      // pinPulseA
  26,      // pinPulseB
  15,      // modulePWMPin
  18,      // modulePWMPinNeg
  2,       // pwmChannel0
  3,       // pwmChannel1
  4484,    // R,
  13.28,   // u0
  -6.997,  // u1
  -4.867,  // u2
  0.9404,  // y0
  0.05964  // y1
);

void IRAM_ATTR onTimer() {
  // Code to execute in the interrupt
  portENTER_CRITICAL_ISR(&timerMux);
  motoRigth.motorRun();
  motoLeft.motorRun();
  portEXIT_CRITICAL_ISR(&timerMux);
}


//******************************************************************
//                             SETUP
//******************************************************************

void setup() {
  
  _setupMotors();
   _setupServos();
   _setupImu();


}


// ****************** SETUP FUNCTIONS ******************

// Setup Servos
void _setupServos() {
  ledcSetup(ChannelServoAzul, freq, resolution);
  ledcAttachPin(PWMPinAzul, ChannelServoAzul);
   ledcSetup(ChannelServoRojo, freq, resolution);
   ledcAttachPin(PWMPinRojo, ChannelServoRojo);  
   servoRojo.setAngle(80, 'r');
  servoAzul.setAngle(90, 'b');
}

// Setup Motors
void _setupMotors() {
  motoRigth.begin();
  motoLeft.begin();
  motoRigth.setVelocity(6);
  motoLeft.setVelocity(6);
}

// Setup IMU
void _setupImu() {
  imuManager.begin();
  // imuManager.calibrate();
}

// Setup Serial
void _setupSerial() {
  Serial.begin(115200);
}

// interruption
void _interruptionSetup(){
  timer = timerBegin(0, 80, true); // timer 0, prescaler 80, count up
  timerAttachInterrupt(timer, &onTimer, true);  
  timerAlarmWrite(timer, 2000, true);
  timerAlarmEnable(timer);
}



void loop() {

  // // // unsigned int current = micros();
  imuManager.update();
  // // //   float imuAngle = imuManager.getCompassAngle();


  // Serial.print(imuManager.getRoll());
  // Serial.print(", ");
  // Serial.println(imuManager.getPitch());


  // servoRojo.setAngle(80, 'r');
  // servoAzul.setAngle(90, 'b');





  motoRigth.motorRun();
  motoLeft.motorRun();


  // // get positions
  // float velR = motoRigth.getAngularVelocity();
  // float velL = motoLeft.getAngularVelocity();

  // float posAngR = motoRigth.getAnglePos();
  // float posAngL = motoLeft.getAnglePos();

  // robotPosition.updatePosition(posAngR, posAngL,4.4);
  // float posRealY = robotPosition.getY();
  // float posRealX = robotPosition.getX();
  // float phiReal = robotPosition.getPhi();

  // // phiReal=0.1*phiReal+0.9*imuAngle;
  // // // debug.runDebug(posRealX, posRealY, posRealY, posRealY);

  // controlTrajectory.update(xTarget, yTarget, posRealX, posRealY, phiReal);
  // float wRigth = controlTrajectory.getWRigth();
  // float wLeft = controlTrajectory.getWLeft();


  // motoRigth.setVelocity(4);
  // motoLeft.setVelocity(4);

  // unsigned long _currentMillisEncoderAngle = millis();  // Actual time Variable Angle
  // if (_currentMillisEncoderAngle - _previousMillisEncoderAngle >= _intervalEncoderAngle) {
  //   _previousMillisEncoderAngle = _currentMillisEncoderAngle;
  //   //   // update real position


  //   angR[ind] = posRealY;
  //   angL[ind] = posRealX;
  //   wR[ind] = phiReal;
  //   wL[ind] = posAngL;

  //   if (ind == N - 1) {
  //     for (int i = 0; i < N; i++) {
  //       Serial.print(angR[i]);
  //       Serial.print(" ");
  //       Serial.print(angL[i]);
  //       Serial.print(" ");
  //       Serial.print(wR[i]);
  //       Serial.print(" ");
  //       Serial.print(wL[i]);
  //       Serial.println();
  //     }
  //     ind = 0;
  //   }
  //   ind++;
  // }



  // // update trajectory
  // controlTrajectory.update(xTarget, yTarget, posRealX, posRealY, phiReal);
  // float wRigth = controlTrajectory.getWRigth();
  // float wLeft = controlTrajectory.getWLeft();

  // // set velocities
  // motoRigth.setVelocity(wRigth);
  // motoLeft.setVelocity(wLeft);









  // Serial.println(motoRigth.getAngularVelocity());
  // Serial.println(motoLeft.getAngularVelocity());
}
