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
  esc3.write(throttle * .64 + 692); //51 - 135 1062 - 1931 869
  esc4.write(throttle); //4-136 579-1937 1358
}

void ESCinit() {
  esc1.attach(3);
  esc2.attach(2);
  esc3.attach(5);
  esc4.attach(4);

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



