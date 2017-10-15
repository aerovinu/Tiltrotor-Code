#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

// The state of the input at a given time.
typedef struct {
  // The throttle value, takes on a value in [0.0, 1.0].
  double throttle;

  // The roll input controlling ailerons, takes on a value in [-1.0, 1.0].
  double roll;

  // The pitch input controlling the elevator, takes on a value in [-1.0, 1.0].
  double pitch;

  // The yaw input controlling the rudder, takes on a value in [-1.0, 1.0].
  double yaw;

} InputState;

// An abstract interface representing a controller.
class AbstractController {
public:
  // Initiatlize the controller with the given pin configuration.
  AbstractController(int throttle_pin, int roll_pin, int pitch_pin, int yaw_pin)
      : throttle_pin_(throttle_pin), roll_pin_(roll_pin),
        pitch_pin_(pitch_pin), yaw_pin_(yaw_pin) {}

  // Polls the controller pins and returns the current input state.
  virtual InputState current_state() = 0;

private:
  int throttle_pin_, roll_pin_, pitch_pin_, yaw_pin_;
}

class Controller : AbstractController {}

#endif
