// Motor A connections
#define ENCA 2
#define ENCB 3
#include <util/atomic.h>


//Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include "RoboClaw.h"

//See limitations of Arduino SoftwareSerial
SoftwareSerial serial(19,18);  
RoboClaw roboclaw(&serial,10000);

#define address 0x80

// globals
long prevT = 0;
int posPrev = 0;
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;
volatile int pos_induct = -999;

// velocity values to be used
float v1Filt = 0;
float v1Prev = 0;


float rpmError = 1;

int desired_rpm = 120;
//pid
float eintegral = 0;
float eprev = 0;

int maxMotorPWM = 117;

void setup() {
  //Open roboclaw serial ports
  roboclaw.begin(38400);
  
  Serial.begin(115200);

  pinMode(ENCA,INPUT_PULLUP);
  pinMode(ENCB,INPUT_PULLUP);
 


  attachInterrupt(digitalPinToInterrupt(ENCA),
                  readEncoder,RISING);


}

void loop() {

  float velocity = fabs(v1Filt);
  // read the position and velocity
  int pos = 0;
  float velocity2 = 0;
  int rotations2 = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = pos_i;
    velocity2 = velocity_i;
  }

  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float velocity1 = (pos - posPrev)/deltaT;
  Serial.print("pos ");
  Serial.println(pos);
  Serial.print("posPrev ");
  Serial.println(posPrev);
  
  posPrev = pos;
  prevT = currT;

  // Convert count/s to RPM
  float v1 = velocity1/600*60.0;
  float v2 = velocity2/600*60.0;
  v1Filt = v1;
  // Low-pass filter (25 Hz cutoff)
  v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  v1Prev = v1;
  //v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev;
  //v2Prev = v2;
  //Serial.print("Desired V ");
  //Serial.print(desired_rpm);
  //Serial.println();
  //Serial.print("RPM ");
  //Serial.print(v1);
  //Serial.println();
  Serial.print("v1 ");
  Serial.println(v1Filt);
  Serial.print("deltaT ");
  Serial.println(deltaT, 7);
  // Compute the control signal u
  // FIND THESE VALUES
  float kp = 0.5;
  float ki = 2.8;
  float kd = 0.0;
  float e = desired_rpm-fabs(v1Filt);
  Serial.print("e ");
  Serial.println(e);
  float dedt = (e-eprev)/deltaT;
  eintegral = eintegral + e*deltaT;
  if (eintegral > 120){
    eintegral = 120;
    }
  float u = kp*e + ki*eintegral + kd*dedt;

  // Set the motor speed and direction
  int dir = -1;
  if (u<0){
    dir = 1;
  }
  int pwr = (int) fabs(u);
  if(pwr > 127){
    pwr = 127;
  }

  Serial.print("dir ");
  Serial.println(dir);
  Serial.print("pwr ");
  Serial.println(pwr);
  spinArm(dir,pwr);
  //roboclaw.BackwardM2(address, pwr);
  
  eprev = e;
  //Serial.print(" ");
  //Serial.print(v1Filt);
  //Serial.print(pos);
  //Serial.println();
  delay(1);
}

void spinArm(int dir, int pwmVal){
  if(dir == 1){ 
    // Turn one way
    roboclaw.ForwardM2(address, pwmVal);
    //roboclaw.ForwardM2(address, 0);
  }
  else if(dir == -1){
    // Turn the other way
    roboclaw.BackwardM2(address,pwmVal);
  } else {
    roboclaw.ForwardM2(address, 0);
    }
}

void readEncoder(){
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCB);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i = pos_i + increment;

}
