# Tiltrotor-Code
All the code for the YUAA tiltrotor project

Priorities:
Process for switching to flying:
-decouple PID for direction of flight to prevent forward accel to be registered as a pitch up
-maintain other l/r PID functions with throttle still controlled by user
-decrease PID effect on throttle linearly with tilt of the main motors
-turn off support motors once tilt is complete
-may have to slow tilting by sending commands to servo stepwise

Flying:
-directly translate transmitter instruction to servos/motors; no need for PID

Process for switching to hover:
-maintain other l/r PID functions with throttle still controlled by user
-increase PID effect on throttle linearly with tilt of the main motors
-may have to slow tilting by sending commands to servo stepwise
-towards the end of the process, recouple PID for direction of flight to prevent forward accel to be registered as a pitch down;
cannot recalibrate accel values otherwise '0' will be a backwards accel

Add emergency stop switch:
-when one of the channels reads HIGH, set all throttles to zero immediately

Add LOS contingency:
-program tiltrotor to continue PID and reduce throttle until downward accel in a signal loss event during hover or transition
-if LOS occurs during flying, set throttles to zero, but maintain servo controls; this will allow the tiltrotor to glide to safety
and allow for maintenance of positive control if the signal is reestablished

Not priority:
-add left/right and forward/backward controls for while in hover mode like quadcopter
-greater autonomy




Questions:
 - During LOS, do all signals go high? Or is checking for one enough?
 - What buttons are we using for transitions?
 - Logging -- will we have an SD card?
