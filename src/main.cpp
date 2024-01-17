/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// LeftMotorC           motor         10              
// LeftMotorA           motor         16              
// LeftMotorB           motor         17              
// RightMotorC          motor         13              
// RightMotorA          motor         19              
// RightMotorB          motor         20              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;
//test

// A global instance of competition
competition Competition;
// define your global instances of motors and other devices here
/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/
void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  wings.set(true);

  inertialSensor.calibrate();
  while(inertialSensor.isCalibrating()){
    wait(100,msec);
  }

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}
// increase kP until we get steady oscillations
// increase kD 

//settings
double kP =0.01;
double kI = 0.00002;
double kD = 0.001;

double turnkP = 0.106;
double turnkI = 0;
double turnkD = 0.0035;

//autonomous settings

int desiredValue = 200; // in degrees
int desiredTurnValue = 0;
int error; // current sensor value - desired value: positional value
int prevError = 0; // position 20 milliseconds ago
int derivative; // error - prevError (slope of position is speed)
int totalError = 0;
int turnError; // current sensor value - desired value: positional value
int turnPrevError = 0; // position 20 milliseconds ago
int turnDerivative; // error - prevError (slope of position is speed)
int turnTotalError = 0;
float WHEEL_DIAM = 4.125;
float PI = 3.1415;
float GEAR_RATIO = 84.0 / 36.0;
float postSlapperAdjustmentAngle;

int turnDirection = 1;
bool turnLeft = true;

bool resetDriveSensors = false;
// variables modified for use
bool enableDrivePID = true;
int drivePID() {
  while(enableDrivePID) {
    if (resetDriveSensors) {
      resetDriveSensors = false;
      LeftMotor.setPosition(0, degrees);
      RightMotor.setPosition(0, degrees);
      inertialSensor.resetRotation();
    }
    int leftMotorPosition = LeftMotor.position(degrees);
    int rightMotorPosition = RightMotor.position(degrees);

     printf("left motor pos is %i\n", leftMotorPosition);
     printf("right motor pos is %i\n", rightMotorPosition);
    // ------------------
    // lateral movement pid
    // ------------------
    // get average of the motors
    int averagePosition = (leftMotorPosition + rightMotorPosition) / 2;
    //potential
    error = desiredValue - averagePosition;
    // derivative
    derivative = error - prevError;
    // velocity -> position (integral)
    totalError += error; // compound error 

    double lateralMotorPower = (error * kP + derivative * kD + totalError * kI);


    // ------------------
    // TURN PID
    // ------------------
    // get average of the motors
    int turnDifference = leftMotorPosition - rightMotorPosition;
    //potential
    turnError = turnDifference - desiredTurnValue;
    // derivative
    turnDerivative = turnError - turnPrevError;
    // velocity -> position (integral)
    turnTotalError += turnError; // compound error 

    // if (desiredTurnValue >= 0) { // if the desired angle is achieved by turning right,
    //   turnDirection = -1; // make sure we're turning right
    // } else {
    //   turnDirection = 1; // turn left!
    // }

    double turnMotorPower = (turnError * turnkP + turnDerivative * turnkD + turnTotalError * turnkI);

    printf("turn power is %f\n", turnMotorPower);
    printf("turn error is %i\n", turnError);
    printf("inertial rotation value is %f\n", inertialSensor.rotation(deg));
    //printf("direction of turning (1 means left, -1 means right) %i\n", turnDirection);
    printf("-----\n");
    
    RightMotor.spin(forward, lateralMotorPower - turnMotorPower, voltageUnits::volt); // always turning left
    LeftMotor.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);
    
    prevError = error;
    turnPrevError = turnError;
    vex::task::sleep(20); // don't want this using up all of our cpu
  }
  return 1;
}

double inchesToDegrees(double inches) {
  return (inches / (WHEEL_DIAM * PI)) * 360 * GEAR_RATIO;
}

double angleTurnToZero(double inputAngle) {
  return (inputAngle > 180 ? 360 - inputAngle : inputAngle * -1);
  // if the heading is left of 0, for ex 355, then we get an output of 5 degrees (so we need to turn 5 degrees right to adjust)
  // if the heading is right of 0, for ex 15, then we get an output of -15 degrees (so we need to turn 15 degrees left to adjust)
}

void autonomous(void) {
  //Catapult.spinFor();
  
  
  vex::task callTask(drivePID); // callTask can be used to modify the task - this starts the task
  // SET UP AND PARK
  //vex::task::sleep(2000);
  resetDriveSensors = true;
  desiredValue = 0;
  desiredTurnValue = 0;

  Catapult.setVelocity(40, rpm);
  Catapult.spinFor(forward, 5, sec);

  Catapult.spin(forward);
  waitUntil((int) rotationSensor.angle(deg) % 180 >= 0 && (int) rotationSensor.angle(deg) % 180 <= 5);
  Catapult.stop();


  resetDriveSensors = true;
  desiredValue = -3;
  desiredTurnValue = 0;
 
  vex::task::sleep(2000);
  resetDriveSensors = true;
  desiredValue = 0;
  desiredTurnValue = -44;

  vex::task::sleep(2000);
  resetDriveSensors = true;
  desiredValue = inchesToDegrees(-80);
  desiredTurnValue = 0;

  vex::task::sleep(5000);
  resetDriveSensors = true;
  desiredValue = 0;
  desiredTurnValue = -73;

  vex::task::sleep(1000);
  resetDriveSensors = true;
  desiredValue = inchesToDegrees(50);
  desiredTurnValue = 0;

  vex::task::sleep(1000);
  resetDriveSensors = true;
  desiredValue = 0;
  desiredTurnValue = -110;

  wings.set(false);
  vex::task::sleep(1000);
  resetDriveSensors = true;
  desiredValue = inchesToDegrees(26);
  desiredTurnValue = 0;

  vex::task::sleep(1000);
  wings.set(true);
  resetDriveSensors = true;
  desiredValue = inchesToDegrees(-24);
  desiredTurnValue = 0;

  vex::task::sleep(1000);
  resetDriveSensors = true;
  desiredValue = 0;
  desiredTurnValue = 90;

  vex::task::sleep(1000);
  resetDriveSensors = true;
  desiredValue = inchesToDegrees(18);
  desiredTurnValue = 0;
  
  vex::task::sleep(1000);
  resetDriveSensors = true;
  desiredValue = 0;
  desiredTurnValue = -90;

  wings.set(false);
  vex::task::sleep(1000);
  
  resetDriveSensors = true;
  desiredValue = inchesToDegrees(28);
  desiredTurnValue = 0;

}
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.
    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}
//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  // Run the pre-autonomous function.
  pre_auton();
  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}