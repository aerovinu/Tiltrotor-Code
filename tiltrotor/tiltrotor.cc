#include "Arduino.h"
#include "controller.h"

Controller controller(0, 0, 0, 0);

void setup() {

}

void loop() {
  InputState s = controller.get_state();
}
