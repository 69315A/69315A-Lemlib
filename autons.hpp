#pragma once

#include "pros/motors.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"

// Motor constructors
extern pros::Motor intake;
extern pros::MotorGroup lb;

// Pneumatic constructors
extern pros::adi::Pneumatics mogoL;
extern pros::adi::Pneumatics mogoR;
extern pros::adi::Pneumatics doinkerL;
extern pros::adi::Pneumatics doinkerR;

// Chassis constructor
extern lemlib::Chassis chassis;

// Sensor constructors
extern pros::Optical oSensor;
extern pros::Rotation rSensor;

// Task flags
extern bool lbTaskEnabled;

// Movement function declarations
void extendMogo();
void retractMogo();
void nextState();

// Match autons
void R_P_ringrush();
void B_P_ringrush();

void R_N_ringrush();
void B_N_ringrush();

void B_N_halfSAWP();
void R_N_halfSAWP();

// Skills
void skills();

// Testing routines
void turnTest();
