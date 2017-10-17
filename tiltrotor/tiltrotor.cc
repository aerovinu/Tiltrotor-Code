#include "Arduino.h"
#include "controller.h"

void setup();
void loop();
void loop_hover();
void loop_transition_flying();
void loop_fly();
void loop_transition_hovering();

typedef enum {
  // In hover mode
  STATE_HOVER = 0,

  // Transitioning to flying mode
  STATE_TRANSITION_FLYING,

  // In flying mode
  STATE_FLY,

  // Transitioning to hover mode
  STATE_TRANSIITON_HOVERING
} STATE;

STATE state = STATE_HOVER;
Controller controller(0, 0, 0, 0);

void setup() {

}

void loop() {
  switch (state) {
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
