#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/asset.hpp"

// controller
extern pros::Controller controller;

//motor groups
extern std::vector<int8_t> lemLeftMotors;
extern std::vector<int8_t> lemRightMotors;
extern std::vector<int> ezLeftMotors;
extern std::vector<int> ezRightMotors;
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;

//sensors
extern pros::Imu imu;
extern pros::Rotation horizontalEnc;
extern pros::Rotation verticalEnc;
extern lemlib::TrackingWheel horizontal;
extern lemlib::TrackingWheel vertical;

//constructors
extern ez::Drive EZchassis;
extern lemlib::Drivetrain drivetrain;
extern lemlib::ControllerSettings linearController;
extern lemlib::ControllerSettings angularController;
extern lemlib::OdomSensors sensors;
extern lemlib::ExpoDriveCurve throttleCurve;
extern lemlib::ExpoDriveCurve steerCurve;
extern lemlib::Chassis LEMchassis;