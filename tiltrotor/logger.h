#ifndef __LOGGER_H__
#define __LOGGER_H__

#include "tiltrotor.h"

class Logger {
public:
  Logger() : _log_rate_ms(0), _last_log_time(0) {};

  /// Will filter logs to only maximum one log every `log_rate_ms` milliseconds.
  /// Set to zero to disable rate limiting.
  void set_log_rate(int log_rate_ms);

  /// Send a tick of the tiltrotor state, given sensor and input states, and
  /// any state-dependent auxiliary information.
  void log_tick(Tiltrotor *tiltrotor, SensorState *ss, InputState *is,
                double aux[], int aux_count);

private:
  int _log_rate_ms;
  double _last_log_time;
};

#endif
