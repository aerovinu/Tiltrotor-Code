#ifndef __PID_H__
#define __PID_H__

#include <vector>

#define INTEGRAL_DURATION (100)

class PIDController {
public:
  PIDController(double Kp, double Ki, double Kd) : Kp_(Kp), Ki_(Ki), Kd_(Kd);

  // Update the PID state with the given error, returning the output throttle.
  // This is expected to be called at a regular time interval.
  double update(double error);

private:
  double Kp_, Ki_, Kd_;
  double[INTEGRAL_DURATION] error_hist_;
  double iters_;
}

class HoverPIDController {
public:
  HoverPIDController() : PIDController(1.0, 1.0, 1.0);
}

#endif
