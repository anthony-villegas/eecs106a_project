// Motor A connections
#define ENCA 2
#define ENCB 3
#define INDUCT 20
int enA = 11;
int in1 = 12;
int in2 = 13;

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

// calculated desired values
float desired_rpm = 80;
float sender_release_pulse_angle = 0;
float catcher_angle = 0;

float rpmError = 1;

//pid
float eintegral = 0;

int maxMotorPWM = 127;

void setup() {
  //Open roboclaw serial ports
  roboclaw.begin(38400);
  
  Serial.begin(115200);

  pinMode(ENCA,INPUT_PULLUP);
  pinMode(ENCB,INPUT_PULLUP);
  pinMode(INDUCT, INPUT);


  attachInterrupt(digitalPinToInterrupt(ENCA),
                  readEncoder,RISING);
  attachInterrupt(digitalPinToInterrupt(INDUCT),
                  senseInductive,RISING);


}

void loop() {

  // read the position and velocity
  int pos = 0;
  noInterrupts(); // disable interrupts temporarily while reading
  pos = pos_i;
  interrupts(); // turn interrupts back on

  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float velocity1 = (pos - posPrev)/deltaT;
  posPrev = pos;
  prevT = currT;

  // Convert count/s to RPM
  float v1 = velocity1/600*60.0;
 
  // Low-pass filter (25 Hz cutoff)
  v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  v1Prev = v1;

  // Compute the control signal u
  // FIND THESE VALUES
  float kp = 2.5;
  float ki = 0;
  float e = desired_rpm-v1Filt;
  eintegral = eintegral + e*deltaT;
  
  float u = kp*e + ki*eintegral;

  // Set the motor speed and direction
  int dir = 1;
  if (u<0){
    dir = -1;
  }
  int pwr = (int) fabs(u);
  if(pwr > maxMotorPWM){
    pwr = maxMotorPWM;
  }
  spinArm(dir,pwr);

  //Serial.print(" ");
  Serial.print(v1Filt);
  //Serial.print(pos);
  Serial.println();
  //delay(1);
}

void spinArm(int dir, int pwmVal){
  roboclaw.BackwardM2(address, pwmVal);
  //if(dir == 1){ 
    // Turn one way
    //roboclaw.BackwardM2(address, pwmVal);
  //}
  //else if(dir == -1){
    // Turn the other way
    //roboclaw.ForwardM2(address,pwmVal);
  //} else {
    //roboclaw.ForwardM2(address, 0);
    //}
}

void senseInductive(){
  int pos = 0;
  noInterrupts(); // disable interrupts temporarily while reading
  pos = pos_i;
  interrupts(); // turn interrupts back on
  
  if(pos_induct==-999){
    pos_induct = pos;
   } else {
     pos_induct = (pos_induct + pos)/2; 
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
