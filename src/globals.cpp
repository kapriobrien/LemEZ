#include "main.h"
#include "lemlib/asset.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"

#define TRACK_WIDTH 10.25
#define DRIVETRAIN_RPM 450
#define HORIZONTAL_DRIFT 2

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
std::vector<int8_t> lemLeftMotors = {-1, -2, -3}; //LEFT lemlib type motor array
std::vector<int8_t> lemRightMotors = {4, -5, 6}; //RIGHT lemlib type motor array 
std::vector<int> ezLeftMotors = {lemLeftMotors[0], lemLeftMotors[1], lemLeftMotors[2]}; //LEFT ez type motor array 
std::vector<int> ezRightMotors = {lemRightMotors[0], lemRightMotors[1], lemRightMotors[2]}; //RIGHT ez type motor array 
pros::MotorGroup leftMotors(lemLeftMotors, pros::MotorGearset::blue); // left motor group
pros::MotorGroup rightMotors(lemRightMotors, pros::MotorGearset::blue); // right motor group

// Inertial Sensor on port 2
pros::Imu imu(2);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(-20);
lemlib::TrackingWheel horizontal(&horizontalEnc, 2, -6);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed (negative signs don't work due to a pros bug)
pros::Rotation verticalEnc(-19);
lemlib::TrackingWheel vertical(&verticalEnc, 2, -.125);

ez::Drive EZchassis(
    
    ezLeftMotors,    // Left Chassis Ports (negative port will reverse it!)
    ezRightMotors,  // Right Chassis Ports (negative port will reverse it!)

    2,      // IMU Port
    3.25,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450);   // Wheel RPM

lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              TRACK_WIDTH, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              DRIVETRAIN_RPM, // drivetrain rpm is 360
                              HORIZONTAL_DRIFT // horizontal drift is 2. If we had traction wheels, it would have been 8
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
lemlib::Chassis LEMchassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);
