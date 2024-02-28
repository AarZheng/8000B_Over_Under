#include "vex.h"


void default_constants(){
  // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
  chassis.set_drive_constants(12, 1.4, 0.01, 10, 0);
  chassis.set_heading_constants(10, 0.1, 0.03, 0.07, 0);
  chassis.set_turn_constants(12, 0.105, 0.015, 0.03, 15);
  chassis.set_swing_constants(12, 0.3, 0.01, 0.2, 15);

  // Each exit condition set is in the form (settle_error, settle_time, timeout).
  chassis.set_drive_exit_conditions(2, 125, 5000);
  chassis.set_turn_exit_conditions(2.5, 50, 2000);
  chassis.set_swing_exit_conditions(2, 50, 2000);
}

//Elims Auton
void farSideAntiBlock() {
  while(chassis.Gyro.isCalibrating()) {
    wait(20, msec);
  }
  intakeMotor.spin(fwd, 100, pct);
  intakePiston.open();
  wait(100, msec);
  chassis.drive_distance(3);
  chassis.drive_distance(-9.5, 0, 9, 1);
  flaps.open();
  chassis.drive_distance(-12, -40, 8, 4);
  chassis.turn_to_angle(-135, 12, chassis.turn_settle_error, chassis.turn_settle_time, 600);
  intakeMotor.stop();
  flaps.close();
  chassis.drive_distance(4, -50, 6, 12, 500);  
  chassis.drive_distance(-15, -90, 10, 10, 700);
  chassis.drive_distance(3);
  chassis.turn_to_angle(110);
  intakeMotor.spin(reverse, 100, pct);
  wait(100, msec);
  chassis.drive_distance(30, 90, 12, 9, 500);
  intakeMotor.stop();
  chassis.drive_distance(-3.5);
  chassis.turn_to_angle(15); 
  intakeMotor.spin(fwd, 100, pct);
  chassis.drive_distance(17.5, 28, 12, 2); 
  chassis.turn_to_angle(155);
  intakeMotor.spin(reverse, 100, pct);
  chassis.drive_distance(20, 175, 12, 1, 1000);
  chassis.drive_distance(-3);
}



//5 Balls touch bar
void farSide5Ball() {
  //Unique PID constants for a faster program
  chassis.set_drive_exit_conditions(2, 40, 5000);
  chassis.set_turn_exit_conditions(2, 40, 2000);
  while(chassis.Gyro.isCalibrating()) {
    wait(20, msec);
  }
  //Drives forward towards the side of the goal
  intakeMotor.spin(fwd, 100, pct);
  intakePiston.open();
  chassis.drive_distance(11);
  wait(50, msec);
  chassis.turn_to_angle(-5);
  //Descores triball from matchload zone
  armPiston.open();
  chassis.drive_distance(9.5, -50, 10, 11);
  chassis.right_swing_to_angle(-150, 12, chassis.swing_settle_error, chassis.swing_settle_time, 500, chassis.swing_kp, chassis.swing_ki, chassis.swing_kd, 15);
  armPiston.close();
  //Scores two triballs
  intakeMotor.spin(reverse, 25, pct);
  chassis.drive_distance(-5, -70, 10, 9);
  intakeMotor.spin(reverse, 100, pct);
  chassis.drive_distance(40, -100, 12, 12, 1200);
  chassis.drive_distance(-6, -105);
  chassis.turn_to_angle(-146, 12, 2, 0, 2000, 0.15, 0.01, 0.01, 15);
  intakeMotor.spin(fwd, 100, pct);
  chassis.drive_distance(18, -155, 12, 3, 1000);
  chassis.turn_to_angle(-25);
  intakeMotor.spin(reverse, 100, pct);
  chassis.drive_distance(5);
  wait(90, msec);
  chassis.turn_to_angle(-120);
  intakeMotor.spin(fwd, 100, pct);
  chassis.drive_distance(9);
  chassis.turn_to_angle(0);
  intakeMotor.stop();
  flaps.open();
  chassis.drive_distance(100, 0, 12, 0, 700);
  chassis.right_swing_to_angle(90);
  flaps.close();
  chassis.drive_distance(18, 170, 12, 9);
  chassisLeft.stop(hold);
  chassisRight.stop(hold);
}

//For when the opponent actually has an auton
void farSide4Ball() {
  chassis.set_drive_exit_conditions(2, 0, 5000);
  chassis.set_turn_exit_conditions(2, 0, 2000);
  while(chassis.Gyro.isCalibrating()) {
    wait(20, msec);
  }
  intakeMotor.spin(fwd, 100, pct);
  intakePiston.open();
  wait(100, msec);
  chassis.drive_distance(3);
  chassis.drive_distance(-8, 5, 10, 1);
  chassis.turn_to_angle(179);
  armPiston.open();
  chassis.drive_distance(12, 130, 10, 12, 800);
  chassis.right_swing_to_angle(70, 12, chassis.swing_settle_error, chassis.swing_settle_time, 400, chassis.swing_kp, chassis.swing_ki, chassis.swing_kd, 15);
  armPiston.close();
  chassis.drive_distance(-7, 130, 10, 9);
  intakeMotor.spin(reverse, 100, pct);
  chassis.drive_distance(100, 85, 12, 9, 1300);
  chassis.drive_distance(-6, 85);
  chassis.drive_distance(100, 85, 12, 0, 300);
  chassis.drive_distance(-4);
  chassis.turn_to_angle(35, 12, 2, 0, 2000, 0.15, 0.01, 0.01, 15);
  intakeMotor.spin(fwd, 100, pct);
  chassis.drive_distance(18, 20, 12, 3, 1000);
  chassis.turn_to_angle(140);
  intakeMotor.stop();
  chassis.drive_distance(100, 180, 12, 12, 900);
  chassis.right_swing_to_angle(-70);
  flaps.close();
  chassis.drive_distance(16, -33, 12, 12);
}

//Near side autonomous
void nearSide() {
  chassis.set_turn_constants(12, 0.12, 0.011, 0.03, 15);
  //Waits until the IMU has completed calibrating
  while(chassis.Gyro.isCalibrating()) {
    wait(20, msec);
  }
    //Knocks alliance triball near goal
    flaps.open();
    wait(100, msec);
    flaps.close();
    intakePiston.open();
    intakeMotor.spin(fwd, 100, pct);
    chassis.turn_to_angle(26);
    //Drives to middle to steal a triball
    chassis.drive_distance(18, 15, 12, 11);
    chassis.turn_to_angle(0);
    chassis.drive_distance(-16, 110, 12, 8);
    //Returns to the side of goal and uses back rammer to knock triball into goal
    chassis.right_swing_to_angle(160);
    chassis.drive_distance(-20, 180, 12, 10, chassis.drive_settle_error, chassis.drive_settle_time, 600);
    chassis.drive_distance(3);
    chassis.turn_to_angle(160);
    //Drives forward to retrieve the triball from the matchload zone
    armPiston.open();
    chassis.drive_distance(11.5, 140, 10, 12);
    chassis.turn_to_angle(65);
    armPiston.close();
    chassis.drive_distance(-7, 150, 10, 12);
    chassis.turn_to_angle(110);
    intakeMotor.spin(reverse, 100, pct);
    //Drives underneath horizontal bar
    chassis.drive_distance(16, 90, 10, 5);
}


void skills() {
  while(chassis.Gyro.isCalibrating()) {
    wait(20, msec);
  }
  int balls = 0;
  intakePiston.open();
  timing.reset();
  chassis.drive_distance(-15, 45, 12, 6, 900);
  chassis.drive_distance(5, -65, 6, 12);
  chassis.drive_distance(-7, 0, 12, 0, 500);
  chassis.turn_to_angle(-65);
  while(balls <= 40) {
    if(ballSensor.isNearObject() && ballSensor.color() == green) {
      cataMotor.spin(fwd, 100, pct);
      balls++;
      wait(50, msec);
    }
    else {
      cataMotor.spinTo(75, degrees, true);
      }
  }
  cataMotor.spin(fwd, 100, pct);
  chassis.turn_to_angle(0);
  cataMotor.spinTo(80, degrees, false);
  chassis.drive_distance(35, -45, 12, 8);
  chassis.drive_distance(25, -135, 12, 8, 1500);
  chassis.drive_distance(-5);
  chassis.drive_distance(25, -135, 12, 0, 500);
  chassis.drive_distance(-3);
  chassis.turn_to_angle(135);
  chassis.drive_distance(30, 225, 9, 3);
  flaps.open();
  chassis.turn_to_angle(-25);
  chassis.drive_distance(20, -45, 12, 8, 600);
  flaps.close();
  chassis.drive_distance(-3);
  chassis.turn_to_angle(-25);
  chassis.drive_distance(-15, 225, 12, 8);
  chassis.turn_to_angle(-45);
  flaps.open();
  chassis.drive_distance(20, -45, 12, 0, 800);
  flaps.close();
  chassis.drive_distance(-10);
  chassis.turn_to_angle(-10);
  flaps.open();
  chassis.drive_distance(20, -65, 12, 9, 800);
  flaps.close();
  chassis.drive_distance(-3);
}


void odom_constants(){
  default_constants();
  chassis.drive_max_voltage = 8;
  chassis.drive_settle_error = 3;
}

void drive_test(){
  chassis.drive_distance(3);
  chassis.drive_distance(6);
  chassis.drive_distance(7);
  chassis.drive_distance(-18);
}

void turn_test(){
  chassis.turn_to_angle(5);
  chassis.turn_to_angle(30);
  chassis.turn_to_angle(90);
  chassis.turn_to_angle(225);
  chassis.turn_to_angle(0);
}

void swing_test(){
  chassis.left_swing_to_angle(90);
  chassis.right_swing_to_angle(0);
}

void full_test(){
  chassis.drive_distance(24);
  chassis.turn_to_angle(-45);
  chassis.drive_distance(-36);
  chassis.right_swing_to_angle(-90);
  chassis.drive_distance(24);
  chassis.turn_to_angle(0);
}

void odom_test(){
  chassis.set_coordinates(0, 0, 0);
  while(1){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(0,50, "X: %f", chassis.get_X_position());
    Brain.Screen.printAt(0,70, "Y: %f", chassis.get_Y_position());
    Brain.Screen.printAt(0,90, "Heading: %f", chassis.get_absolute_heading());
    Brain.Screen.printAt(0,110, "ForwardTracker: %f", chassis.get_ForwardTracker_position());
    Brain.Screen.printAt(0,130, "SidewaysTracker: %f", chassis.get_SidewaysTracker_position());
    task::sleep(20);
  }
}

void tank_odom_test(){
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.turn_to_point(24, 24);
  chassis.drive_to_point(24,24);
  chassis.drive_to_point(0,0);
  chassis.turn_to_angle(0);
}

void holonomic_odom_test(){
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.holonomic_drive_to_point(0, 18, 90);
  chassis.holonomic_drive_to_point(18, 0, 180);
  chassis.holonomic_drive_to_point(0, 18, 270);
  chassis.holonomic_drive_to_point(0, 0, 0);
}

