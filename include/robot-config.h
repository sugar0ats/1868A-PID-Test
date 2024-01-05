using namespace vex;

extern brain Brain;
extern controller Controller1;
extern motor_group LeftMotor;
extern motor_group RightMotor;
extern motor rightmotorA;
extern motor rightmotorM;
extern motor rightmotorB;
extern motor leftmotorA;
extern motor leftmotorM;
extern motor leftmotorB;
extern motor Catapult;
extern motor Intake;
extern digital_out wings;
extern inertial inertialSensor;
extern rotation rotationSensor;
extern digital_out hang;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
