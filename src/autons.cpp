#include "autons.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"

// ANGULAR MOTIONS
// turnToHeading --> turn to certain angle
// turntoPoint --> like turnToHeading but uses coordinates instead 
    // turn >10" of chassis only!!!
// swingToHeading --> turn only using half drivetrain
// swingToPoint --> swingToHeading but coordinates

// LATERAL MOTIONS
// moveToPoint --> move to point w/o specifying degree
// moveToPose --> majority lateral motion, ending with swing to certain heading
    // SLOWER than moveToPoint

void turnTest() {

    nextState();
    pros::delay(1000);

    lbTaskEnabled = false;
    lb.move_relative(1500, 127);
    pros::delay(1000);
    lbTaskEnabled = true;

    nextState();
    
    pros::delay(1000);
    
}

void skills(){

    extendMogo();

    // --------- 1ST CORNER --------- //

    // Score preload on alliance stake
    intake.move_voltage(12000);
    pros::delay(200);

    // Move to first mogo
    chassis.setPose(-60, 0, 90);
    chassis.moveToPoint(-47, 0, 2000);
    chassis.turnToHeading(0, 500);
    chassis.moveToPoint(-47.382, -23.65, 2000, {.forwards = false});
    chassis.waitUntilDone();
    intake.move_voltage(0);

    // Clamp mogo
    retractMogo();
    pros::delay(200);
    intake.move_voltage(-12000);

    // Move to and intake 1st ring
    chassis.moveToPoint(-23.565, -23.444, 2000, {.maxSpeed = 127, .earlyExitRange = 1});
    pros::delay(200);
    intake.move_voltage(12000);

    // Move to and intake 2nd ring
    chassis.moveToPoint(27.735, -49.162, 2000, {.maxSpeed = 100});
    pros::delay(300);

    // Move to center line & set LB loading position
    chassis.moveToPoint(1.3, -38, 2000, {.forwards = false});
    nextState(); // Loading state

    // Turn towards wall stake & set LB ready position
    chassis.turnToHeading(180, 500);

    intake.move_voltage(0);
    intake.move_relative(-150, 127);
    nextState(); // Ready state
    pros::delay(200);
    intake.move_voltage(12000);

    // Move to wall stake and intake 3rd ring
    chassis.moveToPoint(1.3, -61, 2000, {.maxSpeed = 80});
    pros::delay(800);

    nextState(); // Scored state
    pros::delay(100);
    chassis.turnToHeading(185, 500);

    chassis.moveToPoint(-1.3, -47, 2000, {.forwards = false});
    chassis.turnToHeading(270, 1000);
    nextState(); // Resting state

    // Intake 4, 5, 6th rings
    chassis.moveToPoint(-40, -47, 1000, {.maxSpeed = 80, .earlyExitRange = 1});
    chassis.moveToPoint(-60.5, -47, 1000, {.maxSpeed = 50});

    // Intake 7th ring
    chassis.turnToHeading(130, 500);
    chassis.moveToPoint(-43, -58, 1000);
    pros::delay(300);

    // Drop off goal in corner
    chassis.turnToHeading(90, 500);
    chassis.moveToPoint(-57.232, -62.6, 1000, {.forwards = false});
    chassis.waitUntilDone();
    intake.move_voltage(-12000);
    extendMogo();
    pros::delay(500);

    chassis.moveToPoint(-56, -50, 500, {.earlyExitRange = 1});

    // --------- 2ND CORNER --------- //

    // Move to ring
    intake.move_voltage(12000);
    chassis.moveToPoint(31.58, -44.676, 1500, {.maxSpeed = 127, .earlyExitRange = 1});
    chassis.moveToPose(45, -44.7, 90, 1000, {.maxSpeed = 80});

    nextState(); // Loading state

    // Move to and clamp mogo
    chassis.turnToHeading(210, 500);
    chassis.moveToPoint(60.85, -20.597, 2000, {.forwards = false, .maxSpeed = 90});
    chassis.waitUntilDone();

    retractMogo();

    // Move to position for corner
    chassis.moveToPoint(40.5, -42.5, 1000);
    chassis.turnToHeading(310, 1000);
    chassis.waitUntilDone();
    extendMogo();

    // Move forward a bit
    chassis.moveToPoint(34, -36.5, 800);
    chassis.waitUntilDone();
    retractMogo();

    // Back up into corner
    chassis.moveToPoint(58.396, -59.191, 1000, {.forwards = false});
    
    // Turn to 3rd mogo
    chassis.moveToPoint(48.441, -36.33, 2000);
    chassis.turnToHeading(180, 700);
    chassis.waitUntilDone();
    extendMogo();

    // Move to 3rd mogo
    chassis.moveToPoint(48.441, 1, 2000, {.forwards = false, .maxSpeed = 80});      // changed
    chassis.waitUntilDone();
    retractMogo();
    pros::delay(200);

    // Turn to alliance stake
    chassis.turnToHeading(90, 500);
    chassis.moveToPoint(56, 1, 1000);                                                     // changed
    intake.move_voltage(0);
    intake.move_relative(-150, 127);

    // Score alliance stake
    lbTaskEnabled = false;
    intake.move_voltage(-12000);
    lb.move_relative(1750, 127);
    pros::delay(1600);
    lbTaskEnabled = true;

    // Back up
    chassis.moveToPoint(48.441, 0, 1000, {.forwards = false});
    chassis.waitUntilDone();

    nextState(); // Ready state
    nextState(); // Scored state
    nextState(); // Resting state
    intake.move_voltage(12000);

    // --------- 3RD CORNER --------- //

    // Intake 1st ring
    chassis.moveToPoint(23.524, -24.932, 2000);
    chassis.waitUntilDone();
    pros::delay(200);
    intake.move_voltage(0);

    // Move to 3rd quad & intake 2, 3, 4th rings
    chassis.turnToHeading(-45, 500);
    chassis.moveToPoint(-11.539, 12.015, 1000, {.maxSpeed = 100, .earlyExitRange = 1});
    chassis.moveToPoint(-47, 47, 2000, {.maxSpeed = 50});
    pros::delay(500);
    intake.move_voltage(12000);
    chassis.waitUntilDone();
    pros::delay(500);

    // Move to and intake 5th ring
    chassis.turnToHeading(-90, 50);
    chassis.moveToPoint(-60, 47, 1000);
    pros::delay(300);

    // Move to and intake 6th ring
    chassis.moveToPoint(-43.5, 61.5, 1000);
    pros::delay(300);

    // Move to corner and deposit goal
    chassis.turnToHeading(110, 500);
    chassis.moveToPoint(-59, 62.8, 1000, {.forwards = false});
    chassis.waitUntilDone();
    pros::delay(500);
    intake.move_velocity(-12000);
    extendMogo();
    pros::delay(500);

    // --------- 4TH CORNER --------- //

    // Move to 1st ring
    chassis.moveToPoint(-23.683, 47.224, 2000);
    pros::delay(200);
    intake.move_voltage(12000);

    // Turn towards mogo
    chassis.turnToHeading(40, 500);
    intake.move_voltage(0);

    // Move to and clamp 4th mogo
    chassis.moveToPoint(-49.395, 21.229, 2000, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();

    // Clamp mogo
    retractMogo();
    pros::delay(200);
    nextState(); // Loading state

    // Move to wall stake
    chassis.moveToPoint(0, 43.3, 2000);
    intake.move_voltage(12000);
    chassis.waitUntilDone();

    // Turn to wall stake
    chassis.turnToHeading(0, 600);

    // Set LB to ready
    intake.move_voltage(0);
    intake.move_relative(-150, 127);
    nextState(); // Ready state
    pros::delay(200);
    intake.move_voltage(12000);

    // Move to wall stake, score, and intake 1st ring
    chassis.moveToPoint(0, 61.5, 1000, {.maxSpeed = 80});
    chassis.waitUntilDone();

    nextState(); // Scored state
    pros::delay(800);

    // Back up
    chassis.moveToPoint(0, 46.985, 1000, {.forwards = false});
    chassis.waitUntilDone();
    nextState();

    // Turn and move to 2nd ring
    chassis.turnToHeading(90, 500);
    chassis.moveToPoint(23.524, 46.985, 1000);
    chassis.waitUntilDone();
    pros::delay(200);

    // Turn and move to 3rd ring
    chassis.turnToHeading(180, 500);
    chassis.moveToPoint(23.524, 23.501, 1000);
    chassis.waitUntilDone();
    pros::delay(200);

    // Turn and move to 4th ring
    chassis.turnToHeading(55, 500);
    chassis.moveToPoint(47.247, 46.985, 1000, {.maxSpeed = 70, .earlyExitRange = 1});

    // Turn and move to 5th ring
    chassis.turnToHeading(95, 500);
    chassis.moveToPoint(58, 45.013, 1000);
    chassis.waitUntilDone();
    pros::delay(100);

    // Turn and move to 6th ring
    chassis.turnToHeading(325, 500);
    chassis.moveToPoint(46.527, 59.917, 1000);
    chassis.waitUntilDone();
    pros::delay(1000);

    // Turn to corner
    chassis.turnToHeading(240, 500);
    chassis.waitUntilDone();
    intake.move_velocity(-12000);
    extendMogo();

    // Back up into corner & drop off mogo
    chassis.moveToPoint(37.97, 55.595, 1000);
    chassis.waitUntilDone();
    retractMogo();

    chassis.moveToPoint(53.151, 65.713, 1300, {.forwards = false});
    chassis.moveToPoint(37.97, 55.595, 1000);
}

void R_P_ringrush(){

    extendMogo();
    doinkerR.extend();
    pros::delay(200);

    // Doinker top ring and back up
    chassis.setPose(-54, -13, 0);
    chassis.moveToPoint(-54, -27, 2000, {.forwards = false, .maxSpeed = 50});
    chassis.waitUntilDone();

    // Intake 1st ring
    doinkerR.retract();
    intake.move_voltage(12000);
    chassis.moveToPoint(-48, -13, 500);
    
    // Turn to mogo
    chassis.turnToHeading(-67, 500);

    chassis.moveToPoint(-24.058, -23.397, 1500, {.forwards = false});
    intake.move_voltage(0);
    chassis.waitUntilDone();

    // Clamp 2nd mogo
    retractMogo();
    intake.move_voltage(12000);
    pros::delay(200);

    // Move to 2nd ring 
    chassis.turnToHeading(50, 500);
    chassis.moveToPoint(-11, -4, 1000);
    pros::delay(400);
    intake.move_voltage(0);
    chassis.waitUntilDone();

    chassis.turnToHeading(55, 500);
    chassis.waitUntilDone();

    doinkerR.extend();
    pros::delay(400);

    // Move to 3rd ring
    chassis.turnToHeading(74, 500);
    chassis.moveToPoint(-6.25, -5, 500);
    chassis.waitUntilDone();

    doinkerL.extend();
    pros::delay(400);

    // Move back
    chassis.turnToHeading(60, 500);
    chassis.moveToPoint(-37.5, -49, 3000, {.forwards = false});
    chassis.waitUntilDone();
    doinkerR.retract();
    doinkerL.retract();

    // Swing to intake 2, 3, 4th ring
    intake.move_voltage(12000);
    chassis.turnToHeading(70, 500);
    chassis.moveToPoint(-26.6, -45.5, 1000, {.earlyExitRange = 1});

    chassis.moveToPoint(-36.2, -34.7, 1000, {.earlyExitRange = 1});
    chassis.moveToPoint(-60, -40, 1000);
    chassis.waitUntilDone();

    // Turn and move to corner
    chassis.turnToHeading(185, 500);
    chassis.moveToPoint(-60.758, -50, 1000);
    doinkerR.extend();
}

// Working
void B_P_ringrush(){

    extendMogo();
    doinkerL.extend();
    pros::delay(200);

    // Doinker top ring and back up
    chassis.setPose(54, -13, 0);
    chassis.moveToPoint(54, -27, 2000, {.forwards = false, .maxSpeed = 30});
    chassis.waitUntilDone();

    // Intake 1st ring
    doinkerL.retract();
    intake.move_voltage(12000);
    chassis.moveToPoint(48, -13, 500);
    
    // Turn to mogo
    chassis.turnToHeading(60, 500);
    chassis.moveToPoint(24.058, -23.397, 1500, {.forwards = false});
    pros::delay(200);
    intake.move_voltage(0);
    chassis.waitUntilDone();

    // Clamp 2nd mogo
    retractMogo();
    pros::delay(200);
    intake.move_voltage(12000);

    // Move to 2nd ring 
    chassis.turnToHeading(-50, 500);
    chassis.moveToPoint(10, -3, 1000);                  // changed
    pros::delay(300);
    intake.move_voltage(0);
    pros::delay(500);
    intake.move_voltage(-12000);

    chassis.moveToPoint(15, -6, 500, {.forwards = false});      // changed

    chassis.turnToHeading(310, 500);                    // changed
    chassis.waitUntilDone();

    doinkerL.extend();
    pros::delay(400);

    // Move to 3rd ring
    chassis.turnToHeading(285, 500);                        // changed
    chassis.moveToPoint(6.25, -5, 500);
    chassis.waitUntilDone();

    doinkerR.extend();
    pros::delay(400);

    // Move back
    chassis.moveToPoint(37.5, -49, 3000, {.forwards = false});
    chassis.waitUntilDone();
    doinkerR.retract();
    doinkerL.retract();

    // Swing to intake 2, 3, 4th ring
    intake.move_voltage(12000);
    chassis.turnToHeading(290, 500);
    chassis.moveToPoint(27.5, -45.5, 1000, {.earlyExitRange = 1});

    chassis.moveToPoint(36.2, -34.7, 1000, {.earlyExitRange = 1});
    chassis.moveToPoint(60, -40, 1000);
    chassis.waitUntilDone();

    // Turn and move to corner
    chassis.turnToHeading(175, 500);
    chassis.moveToPoint(60.758, -50, 1000);
    doinkerL.extend();
}

void R_N_ringrush(){

}

// unfinished
void B_N_ringrush(){

    extendMogo();
    intake.move_voltage(12000);

    chassis.setPose(51.379, 25.88, 292);

    // Drive to 4 stack
    chassis.moveToPoint(12.186, 41.4, 2000);
    doinkerR.extend();

    // Back up
    chassis.turnToHeading(310, 500);
    chassis.moveToPoint(34, 33.767, 2000, {.forwards = false, .earlyExitRange = 1});
    doinkerR.retract();

    chassis.moveToPose(39, 32.5, 270, 2000, {.forwards = false});
    chassis.waitUntilDone();

    // Drive to and clamp mogo
    chassis.turnToHeading(50, 500);
    chassis.moveToPoint(23.68, 23.822, 1000, {.forwards = false});
    intake.move_voltage(0);
    chassis.waitUntilDone();

    retractMogo();
    pros::delay(200);
    intake.move_voltage(12000);

    // 

    //
    pros::delay(2000);
    intake.move_voltage(0);
}

void B_N_halfSAWP(){

    extendMogo();
    chassis.setPose(51.387, 9.5, 115);
    nextState(); // Loading state

    // Move forward
    chassis.moveToPoint(58.5, 9, 1000);

    // Load preload
    intake.move_voltage(12000); 
    pros::delay(1500);
    intake.move_relative(-250, -127);
    intake.move_voltage(0);

    // Score alliance stake
    lbTaskEnabled = false;
    lb.move_relative(1700, 127);
    pros::delay(1500);
    
    // Move to mogo
    chassis.moveToPoint(22.734, 24.017, 2000, {.forwards = false, .maxSpeed = 90});
    intake.move_voltage(-12000);
    chassis.waitUntilDone();
    retractMogo();
    
    lbTaskEnabled = true;

    nextState(); // Ready state
    nextState(); // Scored state
    nextState(); // Resting state

    pros::delay(500);

    // Turn toward 4-stack
    chassis.turnToHeading(320, 700);
    intake.move_voltage(12000);

    // Move to and intake 1st & 2nd rings
    chassis.moveToPoint(7.5, 38.724, 1500, {.maxSpeed = 80, .earlyExitRange = 1});
    chassis.turnToHeading(0, 500);
    chassis.moveToPoint(7, 50.5, 1000, {.maxSpeed = 80});
    chassis.waitUntilDone();
    pros::delay(200);

    // Move to and intake 3rd ring
    chassis.turnToHeading(100, 700);
    intake.move_velocity(0);
    chassis.waitUntilDone();
    chassis.moveToPoint(25.14, 46.984, 1300);
    intake.move_velocity(12000);
    pros::delay(300);

    // Turn and move to ladder
    chassis.turnToHeading(180, 500);
    chassis.moveToPoint(25.14, 8, 2000, {.maxSpeed = 100});
    chassis.waitUntilDone();
    intake.move_velocity(0);
}

void R_N_halfSAWP(){

    extendMogo();
    chassis.setPose(-51.387, 9.5, 245);
    nextState(); // Loading state

    // Move forward
    chassis.moveToPoint(-58.5, 9, 1000);

    // Load preload
    intake.move_voltage(12000); 
    pros::delay(1500);
    intake.move_relative(-250, -127);
    intake.move_voltage(0);

    // Score alliance stake
    lbTaskEnabled = false;
    lb.move_relative(1700, 127);
    pros::delay(1500);
    
    // Move to mogo
    chassis.moveToPoint(-22.734, 24.017, 2000, {.forwards = false, .maxSpeed = 90});
    intake.move_voltage(-12000);
    chassis.waitUntilDone();
    retractMogo();
    
    lbTaskEnabled = true;

    nextState(); // Ready state
    nextState(); // Scored state
    nextState(); // Resting state

    pros::delay(500);

    // Turn toward 4-stack
    chassis.turnToHeading(45, 700);
    intake.move_voltage(12000);

    // Move to and intake 1st & 2nd rings
    chassis.moveToPoint(-7.5, 38.724, 1500, {.maxSpeed = 80, .earlyExitRange = 1});
    chassis.turnToHeading(0, 500);
    chassis.moveToPoint(-7, 50.5, 1000, {.maxSpeed = 80});
    chassis.waitUntilDone();
    pros::delay(200);

    // Move to and intake 3rd ring
    chassis.turnToHeading(260, 700);
    intake.move_velocity(0);
    chassis.waitUntilDone();
    chassis.moveToPoint(-25.14, 46.984, 1300);
    intake.move_velocity(12000);
    pros::delay(300);

    // Turn and move to ladder
    chassis.turnToHeading(180, 500);
    chassis.moveToPoint(-25.14, 8, 2000, {.maxSpeed = 100});
    chassis.waitUntilDone();
    intake.move_velocity(0);
}