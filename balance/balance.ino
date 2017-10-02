/*Hardware setup for IMU:
MPU9250 Breakout --------- Arduino
VDD ---------------------- 3.3V
SDA ----------------------- A4
SCL ----------------------- A5
GND ---------------------- GND
*/

#include <MPU9250.h>
#include <quaternionFilters.h>
#include <Servo.h> //libraries for ESCs and IMU
#include <Wire.h>

// Maximum throttle coming in from the controller.
#define MAXTHROTTLE 1800

// How far back we keep track of ax/ay values to compute the integral.
#define INTEGRAL_HISTORY 100

// PID coefficients
#define P_COEF 65
#define I_COEF 0.05
#define D_COEF 0.8

void calibrate();
void IMUinit();
void ESCinit();
double clip(double val, double min, double max);

Servo esc1, esc2, esc3, esc4; //declare each esc object MadgwickQuaternionUpdate
MPU9250 IMU;

// Calibration values
double axinit, ayinit, gxinit, gyinit, gzinit, throttleinit;

// Integral histories
double ax_hist[INTEGRAL_HISTORY], ay_hist[INTEGRAL_HISTORY];

// Number of loop() iterations
int iters;

// Keep track of the last 5 estop inputs and only trigger an estop if they are
// all above threshold. This is necessary to prevent fluctuations from
// accidentally triggering an estop.
int estop_hist[5];

// Calibrate the accelerometer/gyro/throttle by measuring the initial values
// (averaging over several seconds). This function blocks for ~3 seconds.
void calibrate() {
  float accelSum[] = { 0, 0 };
  float gyroSum[] = { 0, 0, 0 };
  float throttleSum = 0;
  int accelCount = 0, gyroCount = 0, throttleCount = 0;

  // TODO: Why are accelerometer and gyro calibrated independently?

  while (millis() < 1900) {
    IMU.readAccelData(IMU.accelCount);
    IMU.getAres();
    accelSum[0] += (float)IMU.accelCount[0] * IMU.aRes;
    accelSum[1] += (float)IMU.accelCount[1] * IMU.aRes;
    accelCount++;
  }

  while(millis() < 2200) {
    IMU.readGyroData(IMU.gyroCount);
    IMU.getGres();
    gyroSum[0] += (float)IMU.gyroCount[0] * IMU.gRes;
    gyroSum[1] += (float)IMU.gyroCount[1] * IMU.gRes;
    gyroSum[2] += (float)IMU.gyroCount[2] * IMU.gRes;
    gyroCount++;
  }

  // Keep track of throttle in during calibration, and use this as a baseline
  // during operation.
  while (millis() < 3000) {
    throttleSum += pulseIn(7, HIGH, 25000);
    throttleCount++;
    delay(100);
  }

  axinit = accelSum[0] / accelCount;
  ayinit = accelSum[1] / accelCount;

  gxinit = gyroSum[0] / gyroCount;
  gyinit = gyroSum[1] / gyroCount;
  gzinit = gyroSum[2] / gyroCount;

  throttleinit = throttleSum / throttleCount;
}

void IMUinit() {
  byte c = IMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  if(c == 0x71) {
    Serial.println("MPU9250: Online");
    IMU.MPU9250SelfTest(IMU.SelfTest);
    IMU.calibrateMPU9250(IMU.gyroBias, IMU.accelBias);
    IMU.initMPU9250();
    Serial.println("MPU9250: Initialized");
  } else {
    Serial.println("MPU9250: Connection Failed.");
    Serial.println(c, HEX);
    while(1);
  }
  return 0;
}

void ESCinit() {
  esc1.attach(5, 1000, 2000);
  esc2.attach(2, 1000, 2000);
  esc3.attach(3, 1000, 2000);
  esc4.attach(4, 1000, 2000);

  // esc1.write(2000);
  // esc2.write(2000);
  // esc3.write(2000);
  // esc4.write(2000);

  // esc1.write(0);
  // esc2.write(0);
  // esc3.write(0);
  // esc4.write(0);//esc attachment and low signal, may need more depending on stubbornness of esc calibrations

  return 0;
}

void setup() {
  pinMode(7, INPUT); //pin 7 receives ch3
  pinMode(9, INPUT); //pin 9 receives ch5

  Wire.begin();
  Serial.begin(38400);

  IMUinit();
  ESCinit();

  Serial.print("Calibrating...");
  calibrate();
  Serial.println("done.");
}

void loop() {
  // Check for estop triggered.
  estop_hist[iters % 5] = pulseIn(9, HIGH, 25000);
  bool estop = true;
  for (int i = 0; i < 5; i++) {
    if (estop_hist[i] <= 1600) estop = false;
  }

  if (estop) {
    Serial.println("Triggered emergency shutoff.");
    while(true) {
      esc1.write(0);
      esc2.write(0);
      esc3.write(0);
      esc4.write(0);
    }
  }

  // Get throttle input and scale
  int ch3 = pulseIn(7, HIGH, 25000);
  int throttlein = abs(ch3 - throttleinit) * 1.6 - 1000;

  // if((throttlein > MAXTHROTTLE)) { // connection is lost
  //   Serial.println("Connection lost");
  //   int time0 = millis();
  //   while((millis() - time0) < 1000000) {

  //     if(i == 100) i = 0;
  //     i++; //same as above

  //     store4 = store3;
  //     store3 = store2;
  //     store2 = store1;
  //     store1 = ch5;
  //     ch5 = pulseIn(9, HIGH, 25000);
  //     if((ch5 > 1600)&&(store1 > 1600)&&(store2 > 1600)&&(store3 > 1600)&&(store4 > 1600)){
  //       while(millis() < 1000000000){
  //         esc1.write(550);
  //         esc2.write(550);
  //         esc3.write(550);
  //         esc4.write(550);
  //       }
  //     } ///same as above

  //     IMU.readAccelData(IMU.accelCount);
  //     IMU.getAres();
  //     IMU.ax = (float)IMU.accelCount[0]*IMU.aRes;
  //     IMU.ay = (float)IMU.accelCount[1]*IMU.aRes;
  //     IMU.az = (float)IMU.accelCount[2]*IMU.aRes;
  //     ax = IMU.ax - axinit;
  //     ay = IMU.ay - ayinit; //reads in a values

  //     IMU.readGyroData(IMU.gyroCount);
  //     IMU.getGres();
  //     IMU.gx = (float)IMU.gyroCount[0]*IMU.gRes;
  //     IMU.gy = (float)IMU.gyroCount[1]*IMU.gRes;
  //     IMU.gz = (float)IMU.gyroCount[2]*IMU.gRes;
  //     gx = IMU.gx - gxinit;
  //     gy = IMU.gy - gyinit;
  //     gz = IMU.gz - gzinit; //reads in g values

  //     xvals[i] = ax;
  //     yvals[i] = ay; //stores a values in array
  //     if(millis() < 6600){
  //       xint = findintg(xvals, i);
  //       yint = findintg(yvals, i);
  //     } //passes array to findintg function to find integral

  //     else{
  //       xint = findintg(xvals, 100);
  //       yint = findintg(yvals, 100);
  //     } //once more than 100 values have been stored in array, it no longer cycles the same way

  //     if(throttle5 > 550) throttle5 = 1800 + profact*(ax + ay) + derfact*(gx - gy) + intfact*(xint + yint) - (millis()- time0)*.1;
  //     else throttle5 = 550;
  //     if(throttle2 > 550) throttle2 = 1800 + profact*(ax - ay) + derfact*(-gx - gy) + intfact*(xint - yint) - (millis()- time0)*.1;
  //     else throttle2 = 550;
  //     if(throttle3 > 550)throttle3 = 1800 - profact*(ax + ay) + derfact*(-gx + gy) - intfact*(xint + yint) - (millis()- time0)*.1;
  //     else throttle3 = 550;
  //     if(throttle4 > 550)throttle4 = 1800 - profact*(ax - ay) + derfact*(gx + gy) - intfact*(xint - yint) - (millis()- time0)*.1; //+500 because strangely two separate throttle ranges: 800-2000 and 0-500???
  //     else throttle4 = 550;

  //     esc1.write(throttle5);
  //     esc2.write(throttle2);
  //     esc3.write(throttle3*.64 + 692); //esc3 is calibrated differently; couldn't get it the same
  //     esc4.write(throttle4); //tells copter to stabilize and reduce throttle slowly if control is lost
  //   }
  // }

  IMU.readAccelData(IMU.accelCount);
  IMU.getAres();
  IMU.ax = (float)IMU.accelCount[0] * IMU.aRes;
  IMU.ay = (float)IMU.accelCount[1] * IMU.aRes;
  IMU.az = (float)IMU.accelCount[2] * IMU.aRes;
  double ax = IMU.ax - axinit;
  double ay = IMU.ay - ayinit; //reads in a data

  IMU.readGyroData(IMU.gyroCount);
  IMU.getGres();
  IMU.gx = (float)IMU.gyroCount[0] * IMU.gRes;
  IMU.gy = (float)IMU.gyroCount[1] * IMU.gRes;
  IMU.gz = (float)IMU.gyroCount[2] * IMU.gRes;
  double gx = IMU.gx - gxinit;
  double gy = IMU.gy - gyinit; //reads in g data

  ax_hist[iters % INTEGRAL_HISTORY] = ax;
  ay_hist[iters % INTEGRAL_HISTORY] = ay;

  double axint = 0.0f;
  double ayint = 0.0f;
  int len = iters > INTEGRAL_HISTORY ? INTEGRAL_HISTORY : iters;
  for (int i = 0; i < len; i++) {
    axint += ax_hist[i];
    ayint += ay_hist[i];
  }

  double throttle1 = throttlein + P_COEF * (ax + ay) + D_COEF * (gx - gy) + I_COEF * (axint + ayint);
  double throttle2 = throttlein + P_COEF * (ax - ay) + D_COEF * (-gx - gy) + I_COEF * (axint - ayint);
  double throttle3 = throttlein - P_COEF * (ax + ay) + D_COEF * (-gx + gy) - I_COEF * (axint + ayint);
  double throttle4 = throttlein - P_COEF * (ax - ay) + D_COEF * (gx + gy) - I_COEF * (axint - ayint);

  throttle1 = clip(throttle1, 0, 180);
  throttle2 = clip(throttle2, 0, 180);
  throttle3 = clip(throttle3, 0, 180);
  throttle4 = clip(throttle4, 0, 180);

  esc1.write(throttle1);
  esc2.write(throttle2);
  esc3.write(throttle3);
  esc4.write(throttle4);

  Serial.print("{ ");
  Serial.print(throttle1);
  Serial.print(" \t");
  Serial.print(throttle2);
  Serial.print(" \t");
  Serial.print(throttle3);
  Serial.print(" \t");
  Serial.print(throttle4);
  Serial.println(" }");

  iters++;
}

double clip(double val, double min, double max) {
  return min(max(val, min), max);
}
