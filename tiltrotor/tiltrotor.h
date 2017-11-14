#ifndef __TILTROTOR_H__

#include <Arduino.h>
#include <Servo.h>
#include <MPU9250.h>
#include "PID.h"
#include "controller.h"

// Receiver pins
#define THROTTLE_PIN      (0)
#define ROLL_PIN          (0)
#define PITCH_PIN         (0)
#define YAW_PIN           (0)

// ESC/servo pins
#define MOTOR_LEFT_PIN    (0)
#define MOTOR_RIGHT_PIN   (0)
#define TILT_LEFT_PIN     (0)
#define TILT_RIGHT_PIN    (0)
#define SUPPORT_LEFT_PIN  (0)
#define SUPPORT_RIGHT_PIN (0)
#define AILERON_LEFT_PIN  (0)
#define AILERON_RIGHT_PIN (0)
#define RUDDER_PIN        (0)
#define ELEVATOR_PIN      (0)

// The OP_STATE enum defines the possible states that the tiltrotor can be in.
typedef enum {
  // Waiting for takeoff command
  STATE_OFF = 0,

  // Balancing horizontally, height controlled by throttle. In this state, 0.0
  // throttle will maintain fixed height and 1.0 height is maximum vertical
  // acceleration.
  STATE_TAKEOFF,

  // Transition from STATE_TAKEOFF to STATE_FLY.
  STATE_START_FLYING,

  // Flying horizontally.
  STATE_FLY,

  // Transitioning from STATE_FLY to STATE_LAND.
  STATE_START_LANDING,

  // Balancing horizontally, height controlled by throttle. In this state, 1.0
  // throttle will maintain fixde height and 0.0 height turns off vertical
  // thrust.
  STATE_LAND
} OP_STATE;

typedef struct {
  int accel[3];
  int gyro[3];
} SensorState;

class Tiltrotor {
public:
  Tiltrotor();

  // Get and set the operational state of the tiltrotor
  OP_STATE get_op_state();
  void set_op_state(OP_STATE s);

  // Get the current input state of the receiver
  InputState get_input_state();

  // Get the current sensor state from the IMU
  SensorState get_sensor_state();

  // Sets the throttle of the main wing motors, in range [0.0, 1.0].
  void set_throttle(double throttle);
  void set_throttle(double left_throttle, double right_throttle);

  // Sets the tilt of the motors, ranging from 0.0 (vertical) to 1.0
  // (horizontal).
  void set_tilt_position(double position);
  void set_tilt_position(double left_position, double right_position);

  // Sets the throttle on the back support motors, in range [0.0, 1.0].
  void set_support_throttle(double throttle);
  void set_support_throttle(double throttle_left, double throttle_right);

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

  OP_STATE op_state_;
  Controller controller_;
  MPU9250 imu_;
  SensorState last_sensor_state_;
  Servo motor_left_, motor_right_;
  Servo servo_tilt_left_, servo_tilt_right_;
  Servo motor_support_left_, motor_support_right_;
  Servo servo_aileron_left_, servo_aileron_right_;
  Servo servo_rudder_;
  Servo servo_elevator_;
};

class TiltrotorHoverPIDController : PIDController {
public:
  TiltrotorHoverPIDController() : PIDController(1.0, 1.0, 1.0) {};
};

#endif
