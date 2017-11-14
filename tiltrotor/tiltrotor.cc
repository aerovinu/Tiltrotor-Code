// This is the main file which controls the operation of the tiltrotor.
// High-level operational logic lives here: the main state loops are here, as
// does state transition logic. The low-level commands exposed by the Tiltrotor
// object should be used here for controlling movement.

#include "Arduino.h"
#include "controller.h"
#include "tiltrotor.h"

// This macro will only run a block if it has been more than |ms| milliseconds
// since the last time it was run.
#define RATE_LIMIT(ms) rate_limit_counter += millis() - last_loop_time;        \
    if (rate_limit_counter > ms && (rate_limit_counter -= ms) > 0)

void setup();
void loop();
void loop_off();
void loop_takeoff();
void loop_start_flying();
void loop_fly();
void loop_start_landing();
void loop_land();

// The tiltrotor object communicates with the board directly, exposing functions
// for controlling individual movements.
Tiltrotor tiltrotor;

// The PID controller used for hovering states (STATE_TAKEOFF, STATE_LAND). Must
// be reset before transitioning away from a hovering state.
TiltrotorHoverPIDController hover_pid;

// The time (milliseconds from program start) when the previous loop finished
// executing.
unsigned long last_loop_time = 0;

// You should not need to use this variable. This is used by the RATE_LIMIT
// macro to count how many milliseconds it has been since the rate limited block
// was last called. It must be reset when transitioning states to maintain
// consistency.
unsigned long rate_limit_counter = 0;

void setup() {

}

void loop() {
  switch (tiltrotor.get_op_state()) {
    case STATE_OFF:
    loop_off();
    break;

    case STATE_TAKEOFF:
    loop_takeoff();
    break;

    case STATE_START_FLYING:
    loop_start_flying();
    break;

    case STATE_FLY:
    loop_fly();
    break;

    case STATE_START_LANDING:
    loop_start_landing();
    break;

    case STATE_LAND:
    loop_land();
    break;
  }

  last_loop_time = millis();
}

void loop_off() {
  // Do nothing
}

void loop_takeoff() {
  RATE_LIMIT(100) {

  }
}

void loop_start_flying() {

}

void loop_fly() {

}

void loop_start_landing() {

}

void loop_land() {

}
