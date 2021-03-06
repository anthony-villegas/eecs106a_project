// Pins
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
volatile long eprev = 0;

float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;


// CHANGE FOR TUNING
float desired_rpm = 0;

//pid
float eintegral = 0;

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

  // read the position and velocity
  int pos = 0;
  float velocity2 = 0;
  //noInterrupts(); // disable interrupts temporarily while reading
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = pos_i;
    velocity2 = velocity_i;
  }
  //interrupts(); // turn interrupts back on

  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float velocity1 = (pos - posPrev)/deltaT;
  posPrev = pos;
  prevT = currT;

  // Convert count/s to RPM
  float v1 = velocity1/600*60.0;
  float v2 = velocity2/600*60.0;

  // Low-pass filter (25 Hz cutoff)
  //v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  //v1Prev = v1;
  //v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev;
  //v2Prev = v2;

  // Compute the control signal u
  // FIND THESE VALUES
  float kp = 0;
  //float kp = .7;
  //float ki = 0;
  float ki = 0;
  float kd = 0;
  float e = desired_rpm-v1;
  //float e = desired_rpm-v1;
  //float e = desired_rpm-v1;

  float dedt = (e-eprev)/deltaT;
  
  eintegral = eintegral + e*deltaT;
  if (eintegral > 30){
    eintegral = 30;
    }
  float u = kp*e + kd*dedt + ki*eintegral;

  int pwr = (int) fabs(u);
  if(pwr > 127){
    pwr = 127;
  }
  
  roboclaw.BackwardM2(address, pwr);

  Serial.println(v1);
  //Serial.print(pos);
  //Serial.println();
  //delay(1);
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

  // Compute velocity with method 2
  //long currT = micros();
  //float deltaT = ((float) (currT - prevT_i))/1.0e6;
  //velocity_i = increment/deltaT;
  //prevT_i = currT;
}
