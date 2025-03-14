#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "autons.hpp"

pros::MotorGroup leftMotors{{-4, 2, -3}, pros::MotorGearset::blue};
pros::MotorGroup rightMotors{{12, -11, 13}, pros::MotorGearset::blue};

// Motor constructors
pros::Motor intake(-7, pros::MotorGearset::blue);
pros::MotorGroup lb({-9, 8}, pros::MotorGearset::green);

// Pneumatic constructors
pros::adi::Pneumatics mogoL('A', true);
pros::adi::Pneumatics mogoR('B', true);
pros::adi::Pneumatics doinkerL('C', false);
pros::adi::Pneumatics doinkerR('D', false);

// Sensor constructors
pros::Imu imu(1);
pros::Rotation rSensor(19);
pros::Optical oSensor(14);

// controller constructor
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

// Ladybrown states arrays
int autonStates[4] = {150, 12000, 25000, 50000};      // Remember: centidegrees; 150, 12000, 50000
int driverStates[3] = {150, 12000, 50000};

// Ladybrown variables
int currState = 0;
int target = driverStates[currState];  // Default to driver states

// Ladybrown flags
bool isAutonMode = false;
bool lbTaskEnabled = true;

// Depending on flag, cycles through states
void nextState() {
    int numStates = isAutonMode ? 4 : 3;
    currState = (currState + 1) % numStates;
    target = isAutonMode ? autonStates[currState] : driverStates[currState];
}

// Set up Ladybrown PID & controls velocity 
void liftControl() {
    double kp = 0.5;
    lb.move(kp * (target - rSensor.get_position())/100.0);
    lb.set_brake_mode((pros::E_MOTOR_BRAKE_HOLD));
}

// tracking wheels
pros::Rotation horizontalEnc(5);
pros::Rotation verticalEnc(6);
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -3.75); 
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, 0.125); //0.125

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
                                              13, // derivative gain (kD)
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
                                     13, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  11, // minimum output where drivetrain will move out of 127
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
    pros::Task liftControlTask([]{
        while (true) {
            if(lbTaskEnabled){
                liftControl();
            }
            pros::delay(10);
        }
    });
}

void disabled() {}
void competition_initialize() {}

// 1: Skills

// 2: Red Positive Ring Rush
// 3: Blue Positive Ring Rush

// 4: Red Negative Ring Rush
// 5: Blue Negative Ring Rush

// 6: Red Negative Half SAWP
// 7: Blue Negative Half SAWP

void autonomous() {
    isAutonMode = true;
    skills();
}

void opcontrol() {

    bool rDoinkerToggle = false;
    bool lDoinkerToggle = false;

    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);

        isAutonMode = false;

		// Intake controls
        if (controller.get_digital(DIGITAL_R1)){
            intake.move_velocity(12000);         
        } else if (controller.get_digital(DIGITAL_R2)){
            intake.move_velocity(-12000); 
        } else {
            intake.move_velocity(0); 
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
            lb.move(-50);
        } else {
            lb.move(0);
        }   

        // Mogo mech control
        if (controller.get_digital(DIGITAL_L1)){
            extendMogo();
        } else if (controller.get_digital(DIGITAL_L2)){
            retractMogo();
        }

        // Right doinker control
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
            if(rDoinkerToggle == false){
                doinkerR.extend();
                rDoinkerToggle = true;
            } else {
                doinkerR.retract();
                rDoinkerToggle = false;
            }
        }

        // Left doinker control
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
            if(lDoinkerToggle == false){
                doinkerL.extend();
                lDoinkerToggle = true;
            } else {
                doinkerL.retract();
                lDoinkerToggle = false;
            }
        }

        controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A);

        // delay to save resources
        pros::delay(10);
    }
}