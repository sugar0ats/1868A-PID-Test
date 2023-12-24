#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor LeftMotorC = motor(PORT10, ratio6_1, true);
motor LeftMotorA = motor(PORT16, ratio6_1, false);
motor LeftMotorB = motor(PORT17, ratio6_1, false);
motor RightMotorC = motor(PORT13, ratio6_1, false);
motor RightMotorA = motor(PORT19, ratio6_1, true);
motor RightMotorB = motor(PORT20, ratio6_1, true);

// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}