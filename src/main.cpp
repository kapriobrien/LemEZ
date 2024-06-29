#include "main.h"
#include "lemlib/asset.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
std::vector<int8_t> lemLeftMotors = {-11, -12, 13, -14}; //LEFT lemlib type motor array 
std::vector<int8_t> lemRightMotors = {15, -16, 17, 18}; //RIGHT lemlib type motor array 
std::vector<int> ezLeftMotors = {lemLeftMotors[0], lemLeftMotors[1], lemLeftMotors[2], lemLeftMotors[3]}; //LEFT ez type motor array 
std::vector<int> ezRightMotors = {lemRightMotors[0], lemRightMotors[1], lemRightMotors[2], lemRightMotors[3]}; //RIGHT ez type motor array 
pros::MotorGroup leftMotors(lemLeftMotors, pros::MotorGearset::blue); // left motor group
pros::MotorGroup rightMotors(lemRightMotors, pros::MotorGearset::blue); // right motor group

// Inertial Sensor on port 10
pros::Imu imu(2);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(20);
lemlib::TrackingWheel horizontal(&horizontalEnc, 2, -6);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed (negative signs don't work due to a pros bug)
pros::Rotation verticalEnc(19);
lemlib::TrackingWheel vertical(&verticalEnc, 2, -.125);

ez::Drive EZchassis(
    // These are your drive motors, the first motor is used for sensing!
    ezLeftMotors,     // Left Chassis Ports (negative port will reverse it!)
    ezRightMotors,  // Right Chassis Ports (negative port will reverse it!)

    10,      // IMU Port
    3.25,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450);   // Wheel RPM

lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10.25, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

lemlib::ExpoDriveCurve throttleCurve(0, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     3.0 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(0, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  4.5 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

void initialize() {
    
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    EZchassis.initialize();
    EZchassis.opcontrol_curve_default_set(3, 3.5);
    
    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

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
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    
    //chassis.setPose(0.0, 0.0, 0);
    EZchassis.pid_drive_set(10, 100, true);
    EZchassis.pid_wait();
    EZchassis.drive_set(0,0);

    //chassis.moveToPoint(24, 15, 2000);

}

/**
 * Runs in driver control
 */
void opcontrol() {
    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        EZchassis.opcontrol_arcade_standard(ez::SPLIT); // Standard split arcade
        // delay to save resources
        pros::delay(10);    
    }
}
