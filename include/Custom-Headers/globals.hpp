#include "main.h"

//motors
extern pros::Motor leftFront;
extern pros::Motor leftMidFront;
extern pros::Motor leftMidBack;
extern pros::Motor leftBack;
extern pros::Motor rightFront;
extern pros::Motor rightMidFront;
extern pros::Motor rightMidBack;
extern pros::Motor rightBack;

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

//constructors
extern ez::Drive EZchassis;
extern lemlib::Drivetrain drivetrain;
/*extern lemlib::ControllerSettings linearController;
extern lemlib::ControllerSettings;
extern lemlib::OdomSensors sensors;
extern lemlib::ExpoDriveCurve throttleCurve;
extern lemlib::ExpoDriveCurve steerCurve;
extern lemlib::Chassis chassis;*/