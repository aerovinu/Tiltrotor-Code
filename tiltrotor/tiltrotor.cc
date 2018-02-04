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

// These are variables internal to the `RATE_LIMIT` macro that shouldn't need to
// be used directly. `_rl_counter` is a counter of how many milliseconds have
// passed since the last time the `RATE_LIMIT` macro was used. `_rl_last_time`
// is the time at which the `RATE_LIMIT` macro was last used.
unsigned long _rl_counter = 0;
unsigned long _rl_last_time = 0;

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

// Keep a history of the past several throttle inputs to identify disconnections
#define THROTTLE_HIST_SIZE 5
double throttle_hist[THROTTLE_HIST_SIZE];

// The transition states (STATE_START_FLYING and STATE_START_LANDING) require
// precise timing, so we use this variable to keep track of the time (in
// milliseconds from program start) when we started this transition. This must
// be set when transitioning to one of the transition states.
unsigned long transition_start = 0;

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

// The number of milliseconds that this transition should take place over.
#define FLY_TRANSITION_DUR 10 * 1000

// The transition from hover to flying involves the following processes, taking
// place simultaneously.
//
// 1) Transition control away from PID control. Over the transition, PID control
// is reduced linearly (gradually replaced by controller input). This is only
// for left/right stability: the PID is decoupled from front/back control
// instantaneously.
// 2) Dethrottle support motors. The throttle of the support motors is gradually
// scaled down to 0. This is accomplished alongside (1), because controller
// input does not supply any throttle to the support motors.
// 3) Tilt main motors. The main motors are moved from vertical to horizontal
// linearly over the span of the transition.
void loop_start_flying() {
  double transition_progress =
      (millis() - transition_start) / FLY_TRANSITION_DUR;

  // If we are done with the transition, make sure we clean up and then
  // transition away.
  if (transition_progress >= 1.0) {
    tiltrotor.set_support_throttle(0.0f);
    tiltrotor.set_op_state(STATE_FLY);
    return;
  }


  // As we transition to manual control, we interpolate control between the PID
  // controller and the manual input linearly. This is the fraction of control
  // the PID controller has at this time.
  double pid_scaler = 1 - transition_progress;

  // Gradually move the main engine tilt to horizontal.
  tiltrotor.set_tilt_position(transition_progress);

  // Compute support and main motor throttles from PID.
  SensorState ss = tiltrotor.get_sensor_state();
  double front_pid_throttle = hover_pid_motor_left.update(ss.accel[0]);
  double back_pid_throttle = hover_pid_motor_right.update(-ss.accel[0]);

  InputState is = tiltrotor.get_input_state();

  // Interpolate between control and PID for main throttle.
  tiltrotor.set_throttle(
    front_pid_throttle * pid_scaler + is.throttle * (1 - pid_scaler));

  // Controller input does not use support motors, so take PID input. Support
  // throttle will approach 0 as we approach transition completion.
  tiltrotor.set_support_throttle(back_pid_throttle * pid_scaler);

  // Use (scaled) controller input for the remaining motors.
  tiltrotor.set_aileron_position(
    is.roll * transition_progress, -is.roll * transition_progress);
  tiltrotor.set_rudder_position(is.yaw * transition_progress);
  tiltrotor.set_elevator_position(is.pitch * transition_progress);
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
