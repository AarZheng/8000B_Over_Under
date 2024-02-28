#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;
controller Controller;

// VEXcode device constructors


//Chassis
motor leftBack = motor(PORT18, true);
motor leftMid = motor(PORT13, true);
motor leftFront = motor(PORT15, true);
motor rightBack = motor(PORT19, false);
motor rightMid = motor(PORT17, false);
motor rightFront = motor(PORT16, false);

motor_group chassisRight = motor_group(rightBack, rightMid, rightFront);
motor_group chassisLeft = motor_group(leftBack, leftMid, leftFront);

motor cataMotor = motor(PORT4);
motor intakeMotor = motor(PORT12, true);
motor climbMotor1 = motor(PORT14);
motor climbMotor2 = motor(PORT20);

motor_group climbMotors = motor_group(climbMotor1, climbMotor2);

rotation armRotation = rotation(PORT11);
optical ballSensor = optical(PORT1);

//Pneumatics Constructors
pneumatics flaps = pneumatics(Brain.ThreeWirePort.A); 
pneumatics armPiston = pneumatics(Brain.ThreeWirePort.H);
pneumatics intakePiston = pneumatics(Brain.ThreeWirePort.B);
pneumatics brakePiston = pneumatics(Brain.ThreeWirePort.G);

//Timer
timer timing = timer();


bool cataRunning = false;
bool flapsEngaged = false;
bool brakeEngaged = false;
bool armOut = false;
bool intakeOut = false;
int climbSelection = 0; //0 waiting for climb selection, 1 horizontal, 2 vertical
bool climbRunning = false;
bool coastClimb = false;

// VEXcode generated functions


/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
  chassis.Gyro.calibrate();
  ballSensor.objectDetectThreshold(100);
}