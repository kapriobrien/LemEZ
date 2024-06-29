#include "main.h"
#include "lemlib/asset.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"

void initialize() {
    
    pros::lcd::initialize(); // initialize brain screen
    EZchassis.initialize(); // EZ calibrate sensors (MUST BE FIRST)
    LEMchassis.calibrate(false); // LEMLIB calibrate sensors (MUST BE SECOND)
    defaultConstants(); // EZ set constants
    EZchassis.opcontrol_curve_default_set(3, 3.5); // EZ set drive curve
    
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
            pros::lcd::print(0, "X: %f", LEMchassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", LEMchassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", LEMchassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", LEMchassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}


void disabled() {

}

void competition_initialize() {

}

// get a path used for pure pursuit
// this needs to be put outside a function

ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */

void autonomous() {
    
	testAuton();

}

void opcontrol() {
    // controller
    // loop to continuously update motors
    while (true) {
        
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        
        EZchassis.opcontrol_arcade_standard(ez::SPLIT); 
        
        pros::delay(10);    
    }
}
