#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/imu.hpp"
#include "pros/motors.h"

pros::MotorGroup leftMotors{{-4, 2, -3}, pros::MotorGearset::blue};
pros::MotorGroup rightMotors{{12, -11, 13}, pros::MotorGearset::blue};

// Motor constructors
pros::Motor intake(-7, pros::MotorGearset::blue);
pros::MotorGroup lb({-9, 8}, pros::MotorGearset::green);

// Pneumatic constructors
pros::adi::Pneumatics mogoL('A', false);
pros::adi::Pneumatics mogoR('B', false);
pros::adi::Pneumatics doinkerL('C', false);
pros::adi::Pneumatics doinkerR('D', false);

// Sensor constructors
pros::Imu imu(1);
pros::Rotation rSensor(19);
pros::Optical oSensor(14);

pros::Rotation ohSensor(5);
pros::Rotation ovSensor(6);

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Extend mogo function
void extendMogo(){
    mogoL.extend();
    mogoR.extend();
}
  
// Retract mogo function
void retractMogo(){
    mogoL.retract();
    mogoR.retract();
}

// Ladybrown variables
const int NUM_STATES = 3;
int states[NUM_STATES] = {150, 12000, 48000};      // Remember: centidegrees
int currState = 0;
int target = 0;
bool lbTaskEnabled = true;

// Cycles through states array when called
void nextState() {
    currState += 1;
    if (currState == NUM_STATES) {
        currState = 0;
    }
    target = states[currState];
}

// Set up Ladybrown PID & controls velocity 
void liftControl() {
    double kp = 0.5;
    lb.move(kp * (target - rSensor.get_position())/100.0);
    lb.set_brake_mode((pros::E_MOTOR_BRAKE_HOLD));
}

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(5);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(6);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -3.25);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, 0);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10.75, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              55, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              300, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              17, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors, &throttleCurve, &steerCurve);

void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });

	// Ladybrown Lift Task
    // pros::Task liftControlTask([]{
    //     while (true) {
    //         if(lbTaskEnabled){
    //             liftControl();
    //         }
    //         pros::delay(10);
    //     }
    // });
}

void disabled() {}
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

void autonomous() {}

void opcontrol() {
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);

		// Intake controls
        if (controller.get_digital(DIGITAL_R1)){
            intake.move(127);            
        } else if (controller.get_digital(DIGITAL_R2)){
            intake.move(-70);
        } else {
            intake.move(0);
        }

        // LB lift task control
        if (controller.get_digital_new_press(DIGITAL_RIGHT)) {
            lbTaskEnabled = true;
            nextState();
            pros::delay(20);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
            lbTaskEnabled = false;
            lb.move(127);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
            lbTaskEnabled = false;
            lb.move(-127);
        } else {
            lb.move(0);
        }   

        // Mogo mech control
        if (controller.get_digital(DIGITAL_L1)){
            extendMogo();
        } else if (controller.get_digital(DIGITAL_L2)){
            retractMogo();
        }

        // delay to save resources
        pros::delay(10);
    }
}