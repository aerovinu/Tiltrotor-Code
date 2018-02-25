#include "logger.h"
#include "ArduinoJson.h"

void Logger::set_log_rate(int log_rate_ms) {
  if (log_rate_ms < 0) log_rate_ms = 0;
  _log_rate_ms = log_rate_ms;
}

void Logger::log_tick(Tiltrotor *tiltrotor, SensorState *ss, InputState *is,
                      double aux[], int aux_count) {
  if (_log_rate_ms > 0) {
    if (millis() - _last_log_time < _log_rate_ms) return;
  }

  StaticJsonBuffer<1024> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();

  root["t"]  = millis();

  if (tiltrotor != NULL) {
    root["op"] = tiltrotor->get_op_state();
  }

  if (ss != NULL) {
    JsonArray& sensorArray = root.createNestedArray("ss");
    sensorArray.add(ss->accel[0]);
    sensorArray.add(ss->accel[1]);
    sensorArray.add(ss->accel[2]);
    sensorArray.add(ss->gyro[0]);
    sensorArray.add(ss->gyro[1]);
    sensorArray.add(ss->gyro[2]);
  }

  if (is != NULL) {
    JsonArray& inputArray = root.createNestedArray("is");
    inputArray.add(is->throttle);
    inputArray.add(is->roll);
    inputArray.add(is->pitch);
    inputArray.add(is->yaw);
  }

  if (aux_count > 0) {
    JsonArray& auxArray = root.createNestedArray("aux");
    for (int i = 0; i < aux_count; i++) {
      auxArray.add(aux[i]);
    }
  }

  root.printTo(Serial);

  _last_log_time = millis();
}
