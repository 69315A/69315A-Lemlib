#pragma once
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "robodash/api.h"

// Motor constructors
extern pros::Motor intake;
extern pros::MotorGroup lb;

// Pneumatic constructors
extern pros::adi::Pneumatics mogoL;
extern pros::adi::Pneumatics mogoR;
extern pros::adi::Pneumatics doinkerL;
extern pros::adi::Pneumatics doinkerR;
extern pros::adi::Pneumatics hang;

// Chassis constructor
extern lemlib::Chassis chassis;

// Sensor constructors
extern pros::Optical oSensor;
extern pros::Rotation rSensor;

extern rd::Console brain;

// Task flags
extern bool lbTaskEnabled;
extern int targetHue;

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
void emergency();


// Negatives
void NEW_B_N_qual();
void NEW_R_N_qual();

// Positives
void NEW_B_P_qual();
void NEW_R_P_qual();

void B_P_goalrush();
