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

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

// increase kP until we get steady oscillations
// increase kD 

//settings
double kP = 0.00004;
double kI = 0.0;
double kD = 0.0;

double turnkP = 0.0;
double turnkI = 0.0;
double turnkD = 0.0;

//autonomous settings
int desiredValue = 200; // in degrees
int desiredTurnValue = 300;

int error; // current sensor value - desired value: positional value
int prevError = 0; // position 20 milliseconds ago
int derivative; // error - prevError (slope of position is speed)
int totalError = 0;


int turnError; // current sensor value - desired value: positional value
int turnPrevError = 0; // position 20 milliseconds ago
int turnDerivative; // error - prevError (slope of position is speed)
int turnTotalError = 0;

bool resetDriveSensors = false;

// variables modified for use
bool enableDrivePID = true;

int drivePID() {
  while(enableDrivePID) {

    if (resetDriveSensors) {
      resetDriveSensors = false;

      LeftMotor.setPosition(0, degrees);
      RightMotor.setPosition(0, degrees);
    }

    int leftMotorPosition = LeftMotor.position(degrees);
    int rightMotorPosition = RightMotor.position(degrees);

    // ------------------
    // lateral movement pid
    // ------------------

    // get average of the motors
    int averagePosition = (leftMotorPosition + rightMotorPosition) / 2;

    //potential
    error = averagePosition - desiredValue;

    // derivative
    derivative = error - prevError;

    // velocity -> position (integral)
    // totalError += error; // compound error 

    double lateralMotorPower = (error * kP + derivative * kD);

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
    // turnTotalError += turnError; // compound error 

    double turnMotorPower = (turnError * turnkP + turnDerivative * turnkD );


    LeftMotor.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);
    RightMotor.spin(forward, lateralMotorPower - turnMotorPower, voltageUnits::volt);
    
    prevError = error;
    turnPrevError = turnError;
    vex::task::sleep(20); // don't want this using up all of our cpu

  }

  return 1;
}

float WHEEL_DIAM = 4.125;
float PI = 3.1415;


void emma_inertial_drive_forward(float target) {
  float x = 0.0;
  float error = target - x;
  float speed = 75.0;
  float accuracy = 0.2;
  float ks = 1.0;
  float yaw = 0.0;

  float lspeed = speed * fabs(error) / error - ks * yaw;
  float rspeed = speed * fabs(error) / error + ks * yaw;

  inertialSensor.setRotation(0.0, deg);
  rightmotorA.setRotation(0.0, rev);

  while (fabs(error) > accuracy) {
    RightMotor.spin(forward);
    LeftMotor.spin(forward);

    x = rightmotorA.position(rev) * PI * WHEEL_DIAM;

    error = target - x; 

    yaw = inertialSensor.rotation(degrees);

    lspeed = speed * fabs(error) / error - ks * yaw;
    rspeed = speed * fabs(error) / error + ks * yaw;
  }

}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................

  vex::task callTask(drivePID); // callTask can be used to modify the task - this starts the task
  resetDriveSensors = true;
  desiredValue = 300;
  desiredTurnValue = 600;

  vex::task::sleep(1000);
  resetDriveSensors = true;
  desiredValue = 600;
  desiredTurnValue = -600;


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
