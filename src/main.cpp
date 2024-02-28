#include "vex.h"

using namespace vex;
competition Competition;

/*---------------------------------------------------------------------------*/
/*                             VEXcode Config                                */
/*                                                                           */
/*  Before you do anything else, start by configuring your motors and        */
/*  sensors using the V5 port icon in the top right of the screen. Doing     */
/*  so will update robot-config.cpp and robot-config.h automatically, so     */
/*  you don't have to. Ensure that your motors are reversed properly. For    */
/*  the drive, spinning all motors forward should drive the robot forward.   */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                             JAR-Template Config                           */
/*                                                                           */
/*  Where all the magic happens. Follow the instructions below to input      */
/*  all the physical constants and values for your robot. You should         */
/*  already have configured your robot manually with the sidebar configurer. */
/*---------------------------------------------------------------------------*/

Drive chassis(

//Specify your drive setup below. There are eight options:
//ZERO_TRACKER_NO_ODOM, ZERO_TRACKER_ODOM, TANK_ONE_ENCODER, TANK_ONE_ROTATION, TANK_TWO_ENCODER, TANK_TWO_ROTATION, HOLONOMIC_TWO_ENCODER, and HOLONOMIC_TWO_ROTATION
//For example, if you are not using odometry, put ZERO_TRACKER_NO_ODOM below:
TANK_ONE_ENCODER,

//Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
//You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".

//Left Motors:
motor_group(leftBack, leftMid, leftFront),

//Right Motors:
motor_group(rightBack,rightMid, rightFront),

//Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
PORT5,

//Input your wheel diameter. (4" omnis are actually closer to 4.125"):
2.75,

//External ratio, must be in decimal, in the format of input teeth/output teeth.
//If your motor has an 84-tooth gear and your wheel has a 60-tooth gear, this value will be 1.4.
//If the motor drives the wheel directly, this value is 1:
0.75,

//Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
//For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
360,

/*---------------------------------------------------------------------------*/
/*                                  PAUSE!                                   */
/*                                                                           */
/*  The rest of the drive constructor is for robots using POSITION TRACKING. */
/*  If you are not using position tracking, leave the rest of the values as  */
/*  they are.                                                                */
/*---------------------------------------------------------------------------*/

//If you are using ZERO_TRACKER_ODOM, you ONLY need to adjust the FORWARD TRACKER CENTER DISTANCE.

//FOR HOLONOMIC DRIVES ONLY: Input your drive motors by position. This is only necessary for holonomic drives, otherwise this section can be left alone.
//LF:      //RF:    
PORT1,     -PORT2,

//LB:      //RB: 
PORT3,     -PORT4,

//If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
//If this is a rotation sensor, enter it in "PORT1" format, inputting the port below.
//If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
3,

//Input the Forward Tracker diameter (reverse it to make the direction switch):
2.75,

//Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
//For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
//This distance is in inches:
-2,

//Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
1,

//Sideways tracker diameter (reverse to make the direction switch):
-2.75,

//Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
5.5

);

int current_auton_selection = 0;
bool auto_started = false;

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  default_constants();
  chassis.Gyro.calibrate();
  chassis.Gyro.setHeading(0, deg);
  Controller.Screen.clearScreen();
  Controller.Screen.setCursor(1, 1);
  Controller.Screen.print("Gyro calibrated");

  while(auto_started == false){            //Changing the names below will only change their names on the
    Brain.Screen.clearScreen();            //brain screen for auton selection.
    switch(current_auton_selection){       //Tap the brain screen to cycle through autons.
      case 0:
        Brain.Screen.printAt(50, 50, "Drive Test");
        break;
      case 1:
        Brain.Screen.printAt(50, 50, "Drive Test");
        break;
      case 2:
        Brain.Screen.printAt(50, 50, "Turn Test");
        break;
      case 3:
        Brain.Screen.printAt(50, 50, "Swing Test");
        break;
      case 4:
        Brain.Screen.printAt(50, 50, "Full Test");
        break;
      case 5:
        Brain.Screen.printAt(50, 50, "Odom Test");
        break;
      case 6:
        Brain.Screen.printAt(50, 50, "Tank Odom Test");
        break;
      case 7:
        Brain.Screen.printAt(50, 50, "Holonomic Odom Test");
        break;
    }
    if(Brain.Screen.pressing()){
      while(Brain.Screen.pressing()) {}
      current_auton_selection ++;
    } else if (current_auton_selection == 8){
      current_auton_selection = 0;
    }
    task::sleep(10);
  }
}

void autonomous(void) {
  auto_started = true;
  switch(current_auton_selection){  
    case 0:
      farSide5Ball(); //This is the default auton, if you don't select from the brain.
      break;        //Change these to be your own auton functions in order to use the auton selector.
    case 1:         //Tap the screen to cycle through autons.
      drive_test();
      break;
    case 2:
      turn_test();
      break;
    case 3:
      swing_test();
      break;
    case 4:
      full_test();
      break;
    case 5:
      odom_test();
      break;
    case 6:
      tank_odom_test();
      break;
    case 7:
      holonomic_odom_test();
      break;
 }
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
  climbMotors.stop(hold);
  timing.reset();
  armPiston.close();
  intakePiston.open();
  brakePiston.open();
  intakeOut = true;
  intakeMotor.setStopping(coast);
  chassisLeft.setStopping(coast);
  chassisRight.setStopping(coast);
  climbMotors.setStopping(hold);

    //Intake Piston
    Controller.ButtonY.pressed([] {
      if(!intakeOut) {
        intakePiston.close();
      }
      else {
        intakePiston.open();
      }
      intakeOut = !intakeOut;
    });
    
    //Flaps
    Controller.ButtonR2.pressed([] {
      if(!flapsEngaged) {
        flaps.open();
      }
      else {
        flaps.close();
      }
      flapsEngaged = !flapsEngaged;
    });

    Controller.ButtonL2.pressed([] {
      if(!armOut) {
        armPiston.open();
      }
      else {
        armPiston.close();
      }
      armOut = !armOut;
    });

    Controller.ButtonRight.pressed([] {
      if(climbSelection != 0 && armRotation.angle() < 350) {
        climbSelection = 0;
        while(armRotation.angle() >= 5) {
          climbMotors.spin(fwd, 100, pct);
        }
      }
      coastClimb = false;
      climbMotors.stop(hold);
    });
    Controller.ButtonLeft.pressed([] {
      if(!brakeEngaged) {
        brakePiston.close();
        climbMotors.stop(coast);
      }
      else {
        brakePiston.open();
        climbMotors.stop(hold);
      }
      brakeEngaged = !brakeEngaged;
    });
    //Horizontal Climb
    Controller.ButtonUp.pressed([] {
      if(climbSelection == 0) {
        while((armRotation.angle() <= 78 || armRotation.angle() > 340) && !Controller.ButtonRight.pressing()) {
          climbMotors.spin(reverse, 100, pct);
        }
        climbMotors.stop(hold);
        if(!Controller.ButtonRight.pressing())
          climbSelection = 1;
      }
      
    });
    //Vertical Climb 
    Controller.ButtonDown.pressed([] {
      if(climbSelection == 0) {
        while((armRotation.angle() < 185 || armRotation.angle() > 340) && !Controller. ButtonRight.pressing()) {
          climbMotors.spin(reverse, 100, pct);
        }
        climbMotors.stop();
        if(!Controller.ButtonRight.pressing())
          climbSelection = 2;
      }
    });


    // if(cataRotation.angle() < 60 || cataRotation.angle() > 300) {
    //     cataRunning = true;
    //     while(cataRotation.angle() < 60 || cataRotation.angle() > 300) {
    //       cataMotor.spin(fwd, 100, pct);
    //     }
    //     cataMotor.stop(coast);
    //     cataRunning = false;
    //   }
    //   else {
    //     cataMotor.spinFor(500, msec);
    //   }
  

  while (1) {
    if(climbSelection != 0) {
      if(Controller.ButtonUp.pressing()) {
        coastClimb = true;
        climbMotors.spin(reverse, 100, pct);
      }
      else if(Controller.ButtonDown.pressing()) {
        coastClimb = true;
        climbMotors.spin(fwd, 100, pct);
      }
      else {
        if(coastClimb == true)
          climbMotors.stop(coast);
        else
        climbMotors.stop(hold);
      }
    }

    //Intake + Outtake
    if(Controller.ButtonL1.pressing()) {
      intakeMotor.spin(fwd, 100, pct);
    }
    else if(Controller.ButtonR1.pressing()) {
      intakeMotor.spin(reverse, 100, pct);
    }
    else {
      intakeMotor.stop();
    }


    chassis.control_arcade();

    if(timing.time(sec) >= 104.8) {
      brakePiston.close();
    }

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
