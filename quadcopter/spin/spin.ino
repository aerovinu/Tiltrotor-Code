#include <Servo.h>

Servo esc1, esc2, esc3, esc4;
int throttle = 0;

void setup() {

  Serial.begin(9600);
  Serial.setTimeout(500);

  ESCinit();

}

void loop() {
  if (Serial.available()) throttle = Serial.parseInt();
  Serial.println(throttle);
  esc1.write(throttle);
  esc2.write(throttle);
  esc3.write(throttle);
  esc4.write(throttle);
}

void ESCinit() {
  esc1.attach(3, 1000, 2000);
  esc2.attach(2, 1000, 2000);
  esc3.attach(5, 1000, 2000);
  esc4.attach(4, 1000, 2000);

    esc1.write(2000);
    esc2.write(2000);
    esc3.write(2000);
    esc4.write(2000);

    esc1.write(0);
    esc2.write(0);
    esc3.write(0);
    esc4.write(0);//esc attachment and low signal, may need more depending on stubbornness of esc calibrations

  return 0;
}



