#include "Arduino.h"
#include "controller.h"
#include "tiltrotor.h"

void setup();
void loop();
void loop_off();
void loop_takeoff();
void loop_start_flying();
void loop_fly();
void loop_start_landing();
void loop_land();

Controller controller(0, 0, 0, 0);
Tiltrotor tiltrotor;

void setup() {

}

void loop() {
  switch (tiltrotor.get_op_state()) {
    case STATE_OFF:
    loop_off();
    break;

    case STATE_TAKEOFF:
    loop_takeoff();
    break;

    case STATE_START_FLYING:
    loop_start_flying();
    break;

    case STATE_FLY:
    loop_fly();
    break;

    case STATE_START_LANDING:
    loop_start_landing();
    break;

    case STATE_LAND:
    loop_land();
    break;
  }
}

void loop_off() {
  // Do nothing
}

void loop_takeoff() {

}

void loop_start_flying() {

}

void loop_fly() {

}

void loop_start_landing() {

}

void loop_land() {

}
