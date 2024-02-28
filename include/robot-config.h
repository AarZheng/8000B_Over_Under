using namespace vex;

extern brain Brain;
extern controller Controller;

// VEXcode devices
//Chassis
extern motor leftBack;
extern motor leftMid;
extern motor leftFront;
extern motor rightBack;
extern motor rightMid;
extern motor rightFront;

extern motor_group chassisRight;
extern motor_group chassisLeft;
extern motor_group climbMotors;

//Sensors
extern rotation armRotation;
extern optical ballSensor;

//Other motors
extern motor intakeMotor;
extern motor cataMotor;
extern motor climbMotor1;
extern motor climbMotor2;

//Pneumatics
extern pneumatics flaps;
extern pneumatics armPiston;
extern pneumatics intakePiston;
extern pneumatics brakePiston;


//Timer
extern timer timing;

extern bool cataRunning;
extern bool flapsEngaged;
extern bool brakeEngaged;
extern bool armOut;
extern bool intakeOut;
extern int climbSelection;
extern bool climbRunning;
extern bool coastClimb;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );