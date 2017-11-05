#ifndef __TILTROTOR_H__

#include <Servo.h>

typedef enum {
  // In hover mode
  STATE_HOVER = 0,

  // Transitioning to flying mode
  STATE_TRANSITION_FLYING,

  // In flying mode
  STATE_FLY,

  // Transitioning to hover mode
  STATE_TRANSIITON_HOVERING
} STATE;

class Tiltrotor {
public:
  Tiltrotor();

  // Get and set the state of the tiltrotor
  STATE get_state();
  void set_state(STATE s);

  // Sets the throttle of the main wing motors, in range [0.0, 1.0].
  void set_throttle(double throttle);
  void set_throttle(double left_throttle, right_throttle);

  // Sets the tilt of the motors, ranging from 0.0 (vertical) to 1.0
  // (horizontal).
  void set_tilt_position(double position);
  void set_tilt_position(double left_position, right_position);

  // Sets the throttle on the back support motors, in range [0.0, 1.0].
  void set_support_throttle(double throttle);
  void set_support_throttle(double throttle_left, throttle_right);

  // Sets the aileron position on each side, ranging from -1.0 (down) to 1.0
  // (up).
  void set_aileron_position(double position_left, double position_right);

  // Sets the position of the rudder, ranging from -1.0 (left) to 1.0 (right).
  void set_rudder_position(double position);

  // Sets the elevator position, ranging from -1.0 (down) to 1.0 (up).
  void set_elevator_position(double position);
private:
  // Internal function to send a PWM signal to a servo. It takes an unscaled
  // value and converts it to a 0-180 range as expected by |servo|. |low| and
  // |high| are the ends of the possible range of values of |unscaled|.
  void set_servo(Servo servo, double unscaled, double low, double high);

  State state_;
  Servo motor_left_, motor_right_;
  Servo servo_tilt_left_, servo_tilt_right_;
  Servo motor_support_left_, motor_support_right_;
  Servo servo_aileron_left_, servo_aileron_right_;
  Servo servo_rudder_;
  Servo servo_elevator_;
}

#endif
