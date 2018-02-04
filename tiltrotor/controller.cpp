#include "Arduino.h"
#include "controller.h"

InputState Controller::get_state() {
  // TODO: Remap values to the right range
  InputState state;
  state.throttle = pulseIn(throttle_pin_, HIGH);
  state.roll = pulseIn(roll_pin_, HIGH);
  state.pitch = pulseIn(pitch_pin_, HIGH);
  state.yaw = pulseIn(yaw_pin_, HIGH);

  return state;
}
