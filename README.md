# EZ Template + LemLib

## necesseities:

1. when calibrating the chassis in `void initialize();`, be sure to initialize your EZ-Template chassis 
object first, then your LemLib chassis object. the boolean in `LEMchassis.calibrate(false)' makes it so that only EZ-Template
calibrates the IMU to avoid conflicts

2. EZ-Template pid must be turned off before calling any LemLib functions. Call `EZchassis.pid_drive_toggle(false)` before using any LemLib
movement functions, and the same function but to true before using any EZ-Template movement functions.

3. LemLib and EZ-Template movement functions will run asynchronously unless specified. `EZchassis.pid_wait();` and `LEMchassis.waitUntilDone();`
ensure that the previously called movement function will be completed before reading the next line of code.
