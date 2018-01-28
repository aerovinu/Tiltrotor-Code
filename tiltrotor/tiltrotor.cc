// This is the main file which controls the operation of the tiltrotor.
// High-level operational logic lives here: the main state loops are here, as
// is the state transition logic. The low-level commands exposed by the
// Tiltrotor object should be used here for controlling movement.

#include "Arduino.h"
#include "controller.h"
#include "tiltrotor.h"

// This macro will only run a block if it has been more than |ms| milliseconds
// since the last time it was run.
#define RATE_LIMIT(ms) _rl_counter += millis() - _rl_last_time; _rl_last_time =\
    millis(); if (_rl_counter > ms && (_rl_counter -= ms) > 0)

// This macro resets the internal `RATE_LIMIT` variables, and should be called
// when transitioning to a new state (run loop).
#define RATE_LIMIT_RESET() _rl_counter = 0; _rl_last_time = millis()

void setup();
void loop();
void loop_off();
void loop_takeoff();
void loop_start_flying();
void loop_fly();
void loop_start_landing();
void loop_land();
void loop_stopped();

void hover_loop();
bool estop_check();

// The tiltrotor object communicates with the board directly, exposing functions
// for controlling individual movements.
Tiltrotor tiltrotor;

// The PID controller used for hovering states (STATE_TAKEOFF, STATE_LAND). Must
// be reset before transitioning away from a hovering state.
TiltrotorHoverPIDController hover_pid_motor_left, hover_pid_motor_right,
    hover_pid_support_left, hover_pid_support_right;

// The number of loop iterations we have gone through.
unsigned long loop_count = 0;

// These are variables internal to the `RATE_LIMIT` macro that shouldn't need to
// be used directly. `_rl_counter` is a counter of how many milliseconds have
// passed since the last time the `RATE_LIMIT` macro was used. `_rl_last_time`
// is the time at which the `RATE_LIMIT` macro was last used.
unsigned long _rl_counter = 0;
unsigned long _rl_last_time = 0;

// Keep a history of the past several throttle inputs to identify disconnections
#define THROTTLE_HIST_SIZE 5
double throttle_hist[THROTTLE_HIST_SIZE];

void setup() {

}

void loop() {
  InputState is = tiltrotor.get_input_state();
  throttle_hist[loop_count % THROTTLE_HIST_SIZE] = is.throttle;

  // Check for emergency stop
  if (estop_check()) {
    tiltrotor.estop();
  }

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

    case STATE_STOPPED:
    loop_stopped();
    break;
  }

  loop_count += 1;
}

void loop_off() {
  // Do nothing
}

void loop_takeoff() {
  hover_loop();
}

void loop_start_flying() {

}

void loop_fly() {
  InputState is = tiltrotor.get_input_state();

  tiltrotor.set_throttle(is.throttle);
  tiltrotor.set_aileron_position(is.roll, -is.roll);
  tiltrotor.set_rudder_position(is.yaw);
  tiltrotor.set_elevator_position(is.pitch);
}

void loop_start_landing() {

}

void loop_land() {
  hover_loop();
}

void loop_stopped() {

}

void hover_loop() {
  // Only run the loop every 1/10th of a second
  RATE_LIMIT(100) {
    InputState is = tiltrotor.get_input_state();
    SensorState ss = tiltrotor.get_sensor_state();

    // Compute PID update for main and support motors and set new throttle
    tiltrotor.set_throttle(
      is.throttle + hover_pid_motor_left.update(ss.accel[0] + ss.accel[1]),
      is.throttle + hover_pid_motor_right.update(ss.accel[0] - ss.accel[1]));

    tiltrotor.set_support_throttle(
      is.throttle + hover_pid_support_left.update(-ss.accel[0] - ss.accel[1]),
      is.throttle + hover_pid_support_right.update(-ss.accel[0] + ss.accel[1]));
  }
}

bool estop_check() {
  if (loop_count < THROTTLE_HIST_SIZE) return false;

  for (int i = 0; i < THROTTLE_HIST_SIZE; i++) {
    if (throttle_hist[i] < 1.0) return false;
  }

  return true;
}
