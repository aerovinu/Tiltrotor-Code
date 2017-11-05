#include "Arduino.h"
#include "controller.h"
#include "tiltrotor.h"

void setup();
void loop();
void loop_hover();
void loop_transition_flying();
void loop_fly();
void loop_transition_hovering();

Controller controller(0, 0, 0, 0);
Tiltrotor tiltrotor;

void setup() {

}

void loop() {
  switch (tiltrotor.get_state()) {
    case STATE_HOVER:
    loop_hover();
    break;

    case STATE_TRANSITION_FLYING:
    loop_transition_flying();
    break;

    case STATE_FLY:
    loop_fly();
    break;

    case STATE_TRANSIITON_HOVERING:
    loop_transition_hovering();
    break;
  }
}

void loop_hover() {

}

void loop_transition_flying() {

}

void loop_fly() {

}

void loop_transition_hovering() {

}
