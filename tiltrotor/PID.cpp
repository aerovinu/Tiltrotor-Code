#include "PID.h"

double PIDController::update(double error) {
  double prop, integral, deriv;

  error_hist_[iters_ % INTEGRAL_DURATION] = error;

  prop = error;

  integral = 0.0f;
  for (int i = 0; i < iters_; i++) {
    integral += error_hist_[i];
  }

  if (iters > 1) {
    double last_err =
        error_hist_[(iters_ - 1 + INTEGRAL_DURATION) % INTEGRAL_DURATION];
    deriv = error - last_err;
  } else {
    deriv = error;
  }

  iters_++;

  return Kp_ * prop + Ki_ * integral + Kd_ * deriv;
}
