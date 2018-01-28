#include "tiltrotor.h"

Tiltrotor::Tiltrotor() : op_state_(STATE_OFF),
    controller_(THROTTLE_PIN, ROLL_PIN, PITCH_PIN, YAW_PIN) {
  imu_.calibrateMPU9250(imu_.gyroBias, imu_.accelBias);
  imu_.initMPU9250();
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
};

OP_STATE Tiltrotor::get_op_state() {
  return op_state_;
}

void Tiltrotor::set_op_state(OP_STATE s) {
  op_state_ = s;
}

InputState Tiltrotor::get_input_state() {
  return controller_.get_state();
}

SensorState Tiltrotor::get_sensor_state() {
  if (imu_.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
    imu_.readAccelData(last_sensor_state_.accel);
    imu_.readGyroData(last_sensor_state_.gyro);

    last_sensor_state_.accel[0] = last_sensor_state_.accel[0] * imu_.aRes -
        imu_.accelBias[0];
    last_sensor_state_.accel[1] = last_sensor_state_.accel[1] * imu_.aRes -
        imu_.accelBias[1];
    last_sensor_state_.accel[2] = last_sensor_state_.accel[2] * imu_.aRes -
        imu_.accelBias[2];

    last_sensor_state_.gyro[0] = last_sensor_state_.gyro[0] * imu_.gRes -
        imu_.gyroBias[0];
    last_sensor_state_.gyro[1] = last_sensor_state_.gyro[1] * imu_.gRes -
        imu_.gyroBias[1];
    last_sensor_state_.gyro[2] = last_sensor_state_.gyro[2] * imu_.gRes -
        imu_.gyroBias[2];
  }

  return last_sensor_state_;
}

void Tiltrotor::set_throttle(double throttle) {
  if (get_op_state() == STATE_STOPPED) return;
  set_throttle(throttle, throttle);
}

void Tiltrotor::set_throttle(double left_throttle, double right_throttle) {
  if (get_op_state() == STATE_STOPPED) return;
  set_servo(motor_left_, left_throttle, 0.0f, 1.0f);
  set_servo(motor_right_, right_throttle, 0.0f, 1.0f);
}

void Tiltrotor::set_tilt_position(double position) {
  if (get_op_state() == STATE_STOPPED) return;
  set_tilt_position(position, position);
}

void Tiltrotor::set_tilt_position(double left_position, double right_position) {
  if (get_op_state() == STATE_STOPPED) return;
  set_servo(servo_tilt_left_, left_position, 0.0f, 1.0f);
  set_servo(servo_tilt_right_, right_position, 0.0f, 1.0f);
}

void Tiltrotor::set_support_throttle(double throttle) {
  if (get_op_state() == STATE_STOPPED) return;
  set_support_throttle(throttle, throttle);
}

void Tiltrotor::set_support_throttle(
    double left_throttle, double right_throttle) {
  if (get_op_state() == STATE_STOPPED) return;
  set_servo(motor_support_left_, left_throttle, 0.0f, 1.0f);
  set_servo(motor_support_right_, right_throttle, 0.0f, 1.0f);
}

void Tiltrotor::set_aileron_position(
    double left_position, double right_position) {
  if (get_op_state() == STATE_STOPPED) return;
  set_servo(servo_aileron_left_, left_position, -1.0f, 1.0f);
  set_servo(servo_aileron_right_, right_position, -1.0f, 1.0f);
}

void Tiltrotor::set_rudder_position(double position) {
  if (get_op_state() == STATE_STOPPED) return;
  set_servo(servo_rudder_, position, -1.0f, 1.0f);
}

void Tiltrotor::set_elevator_position(double position) {
  if (get_op_state() == STATE_STOPPED) return;
  set_servo(servo_elevator_, position, -1.0f, 1.0f);
}

void Tiltrotor::estop() {
  set_throttle(0.0f);
  set_support_throttle(0.0f);

  set_op_state(STATE_STOPPED);
}

void Tiltrotor::set_servo(
    Servo servo, double unscaled, double low, double high) {
  double scaled = (unscaled - low) / (high - low) * 180.0f;
  servo.write(scaled);
}
