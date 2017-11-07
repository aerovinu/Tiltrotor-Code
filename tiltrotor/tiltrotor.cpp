#include "tiltrotor.h"

OP_STATE Tiltrotor::get_op_state() {
  return op_state_;
}

void Tiltrotor::set_op_state(OP_STATE s) {
  op_state_ = s;
}

InputState Tiltrotor::get_input_state() {
  return controller_.get_state();
}

void Tiltrotor::set_throttle(double throttle) {
  set_throttle(throttle, throttle);
}

void Tiltrotor::set_throttle(double left_throttle, double right_throttle) {
  set_servo(motor_left_, left_throttle, 0.0f, 1.0f);
  set_servo(motor_right_, right_throttle, 0.0f, 1.0f);
}

void Tiltrotor::set_tilt_position(double position) {
  set_tilt_position(position, position);
}

void Tiltrotor::set_tilt_position(double left_position, double right_position) {
  set_servo(servo_tilt_left_, left_position, 0.0f, 1.0f);
  set_servo(servo_tilt_right_, right_position, 0.0f, 1.0f);
}

void Tiltrotor::set_support_throttle(double throttle) {
  set_support_throttle(throttle, throttle);
}

void Tiltrotor::set_support_throttle(
    double left_throttle, double right_throttle) {
  set_servo(motor_support_left_, left_throttle, 0.0f, 1.0f);
  set_servo(motor_support_right_, right_throttle, 0.0f, 1.0f);
}

void Tiltrotor::set_aileron_position(
    double left_position, double right_position) {
  set_servo(servo_aileron_left_, left_position, -1.0f, 1.0f);
  set_servo(servo_aileron_right_, right_position, -1.0f, 1.0f);
}

void Tiltrotor::set_rudder_position(double position) {
  set_servo(servo_rudder_, position, -1.0f, 1.0f);
}

void Tiltrotor::set_elevator_position(double position) {
  set_servo(servo_elevator_, position, -1.0f, 1.0f);
}

void Tiltrotor::set_servo(
    Servo servo, double unscaled, double low, double high) {
  double scaled = (unscaled - low) / (high - low) * 180.0f;
  servo.write(scaled);
}
