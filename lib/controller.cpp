#include "controller.h"

class AbstractController {
  AbstractController(
      int throttle_pin, int roll_pin, int pitch_pin, int yaw_pin) {
    throttle_pin_ = throttle_pin
  }
}

class Controller : AbstractController {
  InputState current_state() {
    // TODO: Remap values to the right range
    InputState state;
    state.throttle = pulseIn(throttle_pin_, HIGH);
    state.roll = pulseIn(roll_pin_, HIGH);
    state.pitch = pulseIn(pitch_pin_, HIGH);
    state.yaw = pulseIn(yaw_pin_, HIGH);
  }
}
