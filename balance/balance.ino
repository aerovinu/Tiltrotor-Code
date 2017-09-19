/*Hardware setup for IMU: 
MPU9250 Breakout --------- Arduino 
VDD ---------------------- 3.3V 
VDDI --------------------- 3.3V 
SDA ----------------------- A4 
SCL ----------------------- A5 
GND ---------------------- GND 
*/ 

#include <MPU9250.h>
#include <quaternionFilters.h>
#include <Servo.h> //libraries for ESCs and IMU 

int findintg(double[], int); //function to find integral 
void IMUinit(); 
void ESCinit(); 

Servo esc5, esc2, esc3, esc4; //declare each esc object 
MPU9250 myIMU; //declare IMU object 
float axinit, ayinit, gxinit, gyinit, gzinit, ax, ay, gx, gy, gz; //accelerometer and gyro vars 
int ch3, ch5, initavg, i = 0, k = 0, store1 = 0, store2 = 0, store3 = 0, store4 = 0; //ch3 is throttle, ch5 is estop switch, initavg is average initial PWM coming from rx, i and k are counters 
const int MAX = 100; //const integer for the size of the arrays that store accelerometer values 
long initsum = 0; //sum of throttle values for calibration 
double xvals[MAX], yvals[MAX]; //arrays to store accelerometer values, 

void setup() { 
  pinMode(7, INPUT); //pin 7 receives ch3 
  pinMode(9, INPUT); //pin 9 receives ch5 
  float axsum = 0, aysum = 0, gxsum = 0, gysum = 0, gzsum = 0; //vars to store sums of initial a and g values for calibration 
  int i = 0, j = 0; //counters 
  
  Wire.begin(); 
  Serial.begin(38400); 
  IMUinit(); //initializes IMU 
  
  while(millis() < 1900) { 
    myIMU.readAccelData(myIMU.accelCount); 
    myIMU.getAres(); 
    axsum = axsum + (float)myIMU.accelCount[0]*myIMU.aRes; 
    aysum = aysum + (float)myIMU.accelCount[1]*myIMU.aRes; 
    i++; 
  } //calibration of IMU taking into account initial angular data - therefore quad should NOT start on an incline; will have to find fix for this eventually 

  while(millis() < 2200) {
    myIMU.readGyroData(myIMU.gyroCount); 
    myIMU.getGres(); 
    gxsum = gxsum + (float)myIMU.gyroCount[0]*myIMU.gRes; 
    gysum = gysum + (float)myIMU.gyroCount[1]*myIMU.gRes; 
    gzsum = gzsum + (float)myIMU.gyroCount[2]*myIMU.gRes; 
    j++; 
  } //same as above for gyro 

  axinit = axsum / i; 
  ayinit = aysum / i; 
  gxinit = gxsum / j; 
  gyinit = gysum / j; 
  gzinit = gzsum / j; 
  
  ESCinit(); //initializes ESCs 
  
  while(millis() < 3000){ 
    ch3 = pulseIn(7, HIGH, 25000); 
    initsum = initsum + ch3; 
    i++; 
    delay(100); 
  } 
  initavg = initsum / i; //finds avg pwm in from ch3 for scale of throttle signals 

} 

void loop() { 

  int throttlein, throttle5 = 551, throttle2 = 551, throttle3 = 551, throttle4 = 551, maxthrottle = 1800, stablethrottle = 1800, minthrottle = 550, xint = 0, yint = 0; 
  double jx = 0, jy = 0, jz = 0, intfact = 0.05, derfact = .8, profact = 65; 
  
  if(i == 100) i = 0; 
  i++; //cycler for a value storing in array 
  
  ch3 = pulseIn(7, HIGH, 25000); 
  throttlein = (abs(ch3 - initavg))*1.6 - 1000; //determines throttlein 
  
  store4 = store3; 
  store3 = store2; 
  store2 = store1; 
  store1 = ch5; 
  ch5 = pulseIn(9, HIGH, 25000); 
  if((ch5 > 1600)&&(store1 > 1600)&&(store2 > 1600)&&(store3 > 1600)&&(store4 > 1600)){ 
  while(millis() < 1000000000){ 
  esc5.write(0); 
  esc2.write(0); 
  esc3.write(0); 
  esc4.write(0); 
  } 
  } //immediately shuts off motors if kill switch is activated 
  
  if((throttlein > maxthrottle)){ //if connection lost 
  Serial.println ("error"); 
  int time0 = millis(); 
  while((millis() - time0) < 1000000){ 
  
  if(i == 100) i = 0; 
  i++; //same as above 
  
  store4 = store3; 
  store3 = store2; 
  store2 = store1; 
  store1 = ch5; 
  ch5 = pulseIn(9, HIGH, 25000); 
  if((ch5 > 1600)&&(store1 > 1600)&&(store2 > 1600)&&(store3 > 1600)&&(store4 > 1600)){ 
    while(millis() < 1000000000){ 
        esc5.write(550); 
        esc2.write(550); 
        esc3.write(550); 
        esc4.write(550); 
      } 
    } ///same as above 
    
    myIMU.readAccelData(myIMU.accelCount); 
    myIMU.getAres(); 
    myIMU.ax = (float)myIMU.accelCount[0]*myIMU.aRes; 
    myIMU.ay = (float)myIMU.accelCount[1]*myIMU.aRes; 
    myIMU.az = (float)myIMU.accelCount[2]*myIMU.aRes; 
    ax = myIMU.ax - axinit; 
    ay = myIMU.ay - ayinit; //reads in a values 
    
    myIMU.readGyroData(myIMU.gyroCount); 
    myIMU.getGres(); 
    myIMU.gx = (float)myIMU.gyroCount[0]*myIMU.gRes; 
    myIMU.gy = (float)myIMU.gyroCount[1]*myIMU.gRes; 
    myIMU.gz = (float)myIMU.gyroCount[2]*myIMU.gRes; 
    gx = myIMU.gx - gxinit; 
    gy = myIMU.gy - gyinit; 
    gz = myIMU.gz - gzinit; //reads in g values 
    
    xvals[i] = ax; 
    yvals[i] = ay; //stores a values in array 
    if(millis() < 6600){ 
      xint = findintg(xvals, i); 
      yint = findintg(yvals, i); 
    } //passes array to findintg function to find integral 
    
    else{ 
      xint = findintg(xvals, 100); 
      yint = findintg(yvals, 100); 
    } //once more than 100 values have been stored in array, it no longer cycles the same way 
    
    if(throttle5 > 550) throttle5 = 1800 + profact*(ax + ay) + derfact*(gx - gy) + intfact*(xint + yint) - (millis()- time0)*.1; 
    else throttle5 = 550; 
    if(throttle2 > 550) throttle2 = 1800 + profact*(ax - ay) + derfact*(-gx - gy) + intfact*(xint - yint) - (millis()- time0)*.1; 
    else throttle2 = 550; 
    if(throttle3 > 550)throttle3 = 1800 - profact*(ax + ay) + derfact*(-gx + gy) - intfact*(xint + yint) - (millis()- time0)*.1; 
    else throttle3 = 550; 
    if(throttle4 > 550)throttle4 = 1800 - profact*(ax - ay) + derfact*(gx + gy) - intfact*(xint - yint) - (millis()- time0)*.1; //+500 because strangely two separate throttle ranges: 800-2000 and 0-500??? 
    else throttle4 = 550; 
    
    esc5.write(throttle5); 
    esc2.write(throttle2); 
    esc3.write(throttle3*.64 + 692); //esc3 is calibrated differently; couldn't get it the same 
    esc4.write(throttle4); //tells copter to stabilize and reduce throttle slowly if control is lost 
    } 
  } 
  
  myIMU.readAccelData(myIMU.accelCount); 
  myIMU.getAres(); 
  myIMU.ax = (float)myIMU.accelCount[0]*myIMU.aRes; 
  myIMU.ay = (float)myIMU.accelCount[1]*myIMU.aRes; 
  myIMU.az = (float)myIMU.accelCount[2]*myIMU.aRes; 
  ax = myIMU.ax - axinit; 
  ay = myIMU.ay - ayinit; //reads in a data 
  
  myIMU.readGyroData(myIMU.gyroCount); 
  myIMU.getGres(); 
  myIMU.gx = (float)myIMU.gyroCount[0]*myIMU.gRes; 
  myIMU.gy = (float)myIMU.gyroCount[1]*myIMU.gRes; 
  myIMU.gz = (float)myIMU.gyroCount[2]*myIMU.gRes; 
  gx = myIMU.gx - gxinit; 
  gy = myIMU.gy - gyinit; //reads in g data 
  
  xvals[i] = ax; 
  yvals[i] = ay; //stores a values in array 
  
  if(millis() < 6600){ 
    xint = findintg(xvals, i); 
    yint = findintg(yvals, i); 
  } 
  
  else{ 
    xint = findintg(xvals, 100); 
    yint = findintg(yvals, 100); 
  } //as described above 
  
  throttle5 = throttlein + profact*(ax + ay) + derfact*(gx - gy) + intfact*(xint + yint); 
  throttle2 = throttlein + profact*(ax - ay) + derfact*(-gx - gy) + intfact*(xint - yint); 
  throttle3 = throttlein - profact*(ax + ay) + derfact*(-gx + gy) - intfact*(xint + yint); 
  throttle4 = throttlein - profact*(ax - ay) + derfact*(gx + gy) - intfact*(xint - yint); //sum of throttle inputs 
  
  if(throttle5 < minthrottle) throttle5 = 550; 
  if(throttle2 < minthrottle) throttle2 = 550; 
  if(throttle3 < minthrottle) throttle3 = 550; 
  if(throttle4 < minthrottle) throttle4 = 550; //prevents overthrottling due to tilt from the weird 0-500 range 
  
  esc5.write(throttle5); 
  esc2.write(throttle2); 
  esc3.write(throttle3*.64 + 692); 
  esc4.write(throttle4); //writes throttle values 
  
  Serial.println(throttle2); 

} 

void IMUinit(){ 
byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250); 
Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX); 
Serial.print(" I should be "); Serial.println(0x71, HEX); 
if(c == 0x71){ 
Serial.println("MPU9250 is online."); 
myIMU.MPU9250SelfTest(myIMU.SelfTest); 
myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias); 
myIMU.initMPU9250(); 
Serial.println("MPU9250 initialized."); 
} //necessary initialization commands 

else{ 
Serial.print("Connection Failed."); 
Serial.println(c,HEX); 
while(1); 
} //confirms IMU initialization 
return 0; 
} 

void ESCinit(){ 
esc5.attach(5); 
esc2.attach(2); 
esc3.attach(3); 
esc4.attach(4); 

esc5.write(2000); 
esc2.write(2000); 
esc3.write(2000); 
esc4.write(2000); 

esc5.write(0); 
esc2.write(0); 
esc3.write(0); 
esc4.write(0);//esc attachment and low signal, may need more depending on stubbornness of esc calibrations 

return 0; 
} 

int findintg(double vals[], int num){ 
int total = 0; 
for(int j = 0; j < num; j++){ 
total = total + 10*vals[j]; 
} 
return total; 
} 

/*throttle5 = throttlein + profact*(ax + ay) + derfact*(gx/(.5*abs(gx)) - gy/(.5*abs(gy))) + intfact*(xint + yint); 
throttle2 = throttlein + profact*(ax - ay) + derfact*(-gx/(.5*abs(gx)) - gy/(.5*abs(gy))) + intfact*(xint - yint); 
throttle3 = throttlein - profact*(ax + ay) + derfact*(-gx/(.5*abs(gx)) + gy/(.5*abs(gy))) - intfact*(xint + yint); 
throttle4 = throttlein - profact*(ax - ay) + derfact*(gx/(.5*abs(gx)) + gy/(.5*abs(gy))) - intfact*(xint - yint); //sum of throttle inputs*/
