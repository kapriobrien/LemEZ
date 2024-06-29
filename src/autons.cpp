#include "main.h"
#include "lemlib/asset.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"

void defaultConstants() {
  // PID Constants
  EZchassis.pid_drive_constants_set(10, 0, 100); // Sets forward and backward
  EZchassis.pid_heading_constants_set(3, 0, 20);
  EZchassis.pid_turn_constants_set(3, 0, 20);
  EZchassis.pid_swing_constants_set(5, 0, 30); // Sets forward and backward

}

void testAuton(){

    EZchassis.drive_brake_set(MOTOR_BRAKE_HOLD); 
    EZchassis.pid_drive_toggle(false);

    LEMchassis.moveToPoint(20, 15, 3000, {true, 60}, true);
    LEMchassis.waitUntilDone();

    EZchassis.pid_drive_toggle(true);
    EZchassis.pid_turn_set(-45, 110, true);
    EZchassis.pid_wait();

    EZchassis.pid_drive_set(25, 110, true);
    EZchassis.pid_wait();

    EZchassis.pid_drive_toggle(false);

}