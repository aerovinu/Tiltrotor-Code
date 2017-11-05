#include "tiltrotor.h"

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

Tiltrotor() {
  state_ = STATE_HOVER;
  motor_left_.attach(MOTOR_LEFT_PIN);
  motor_right_.attach(MOTOR_RIGHT_PIN);
  servo_tilt_left_.attach(TILT_LEFT_PIN);
  servo_tilt_right_.attach(TILT_RIGHT_PIN);
  motor_support_left_.attach(SUPPORT_LEFT_PIN);
  motor_support_right_.attach(SUPPORT_RIGHT_PIN);
  servo_aileron_left_.attach(AILERON_LEFT_PIN);
  servo_aileron_right_.attach(AILERON_RIGHT_PIN);
  servo_rudder_.attach(RUDDER_PIN);
  servo_elevator_.attach(ELEVATOR_PIN);
}

STATE Tiltrotor::get_state() {
  return this.state_;
}

void Tiltrotor::set_state(STATE s) {
  this.state_ = s;
}

void set_throttle(double throttle) {
  this.set_throttle(throttle, throttle);
}

void set_throttle(double left_throttle, double right_throttle) {
  this.set_servo(this.motor_left_, left_throttle, 0.0f, 1.0f);
  this.set_servo(this.motor_right_, right_throttle, 0.0f, 1.0f);
}

void set_tilt_position(double position) {
  this.set_tilt_position(position, position);
}

void set_tilt_position(double left_position, right_position) {
  this.set_servo(this.servo_tilt_left_, left_position, 0.0f, 1.0f);
  this.set_servo(this.servo_tilt_right_, right_position, 0.0f, 1.0f);
}

void set_support_throttle(double throttle) {
  this.set_support_throttle(throttle, throttle);
}

void set_support_throttle(double left_throttle, right_throttle) {
  this.set_servo(this.motor_support_left_, left_throttle, 0.0f, 1.0f);
  this.set_servo(this.motor_support_right_, right_throttle, 0.0f, 1.0f);
}

void set_aileron_position(double left_position, right_position) {
  this.set_servo(this.servo_aileron_left_, left_position, -1.0f, 1.0f);
  this.set_servo(this.servo_aileron_right_, right_position, -1.0f, 1.0f);
}

void set_rudder_position(double position) {
  this.set_servo(this.servo_rudder_, position, -1.0f, 1.0f);
}

void set_elevator_position(double position) {
  this.set_servo(this.servo_elevator_, position, -1.0f, 1.0f);
}

void set_servo(Servo servo, double unscaled, double low, double high) {
  double scaled = (unscaled - low) / (high - low);
  servo.write(scaled);
}
