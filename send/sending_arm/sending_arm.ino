// Pins
#define ENCA 2
#define ENCB 3
#define INDUCT 20
#define servoPin 9

//Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include "RoboClaw.h"

#include <Servo.h>

//See limitations of Arduino SoftwareSerial
SoftwareSerial serial(19,18);  
RoboClaw roboclaw(&serial,10000);

Servo myservo;

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
float v2Filt = 0;
float v2Prev = 0;

// calculated desired values
float desired_rpm = 0;
float sender_release_pulse_angle = 0;
float catcher_angle = 0;

float rpmError = 1;

//pid
float eintegral = 0;

// open and closed angles of servo, CHANGE THESE VALUES
int servoOpen = 150;
int servoClosed = 100;

boolean ball_released = false;

void setup() {
  //Open roboclaw serial ports
  roboclaw.begin(38400);
  
  Serial.begin(115200);

  pinMode(ENCA,INPUT_PULLUP);
  pinMode(ENCB,INPUT_PULLUP);
  pinMode(INDUCT, INPUT);
  // Attach the Servo variable to a pin:
  myservo.attach(servoPin);

  attachInterrupt(digitalPinToInterrupt(ENCA),
                  readEncoder,RISING);
  attachInterrupt(digitalPinToInterrupt(INDUCT),
                  senseInductive,RISING);

  const float g = 9.81;
  const float l = 0.127; // arm length (5")
  const float d = 0.5; // distance to target (m)
  const float theta = 45*PI/180;
  const float theta_zero = PI/2;
  const float d_receiver = cos(theta)*(l-0.0254); // distance from arm center to catcher center
  const float d_total = 2*d_receiver + d;
  const float v_0 = sqrt(g*d_total/sin(2*theta));
  const float desired_omega = v_0/l; //rad/s
  float desired_rpm = desired_omega*60/(2*PI);
  const float t_r = (theta+theta_zero)/desired_omega; // time of release at theta_sum = 135 after t = 0 at theta = 0
  const float t_e = 30/1000; //error time (default set at 30 ms)
  const float t_l = t_r - t_e; //time accounting for delay of signal
  
  const float flight_time = 2*v_0*sin(theta)/g;
  
  float catcher_angle = ((3*PI/2 - PI/4) - desired_omega*flight_time)*180/(PI*0.6);
  
  while (abs(catcher_angle) > 360){
    catcher_angle = catcher_angle + 360;
  }
  
  sender_release_pulse_angle = ((desired_omega*t_l)*180/(PI))/0.6; //0.6 to get which pulse # from 1-600
}

void loop() {
    if (ball_released == true) {
      roboclaw.ForwardM2(address, 0);
      while(true){}
    }
  // read the position and velocity
  int pos = 0;
  float velocity2 = 0;
  noInterrupts(); // disable interrupts temporarily while reading
  pos = pos_i;
  velocity2 = velocity_i;
  interrupts(); // turn interrupts back on

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
  v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  v1Prev = v1;
  v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev;
  v2Prev = v2;

  // Compute the control signal u
  // FIND THESE VALUES
  float kp = 5;
  float ki = 10;
  float e = desired_rpm-v1Filt;
  eintegral = eintegral + e*deltaT;
  
  float u = kp*e + ki*eintegral;

  // Set the motor speed and direction
  int dir = 1;
  if (u<0){
    dir = -1;
  }
  int pwr = (int) fabs(u);
  if(pwr > 127){
    pwr = 127;
  }
  //spinArm(dir,pwr);
  spinArm(-1, 80);
  delay(3000);
  spinArm(-1, 0);
  delay(10000);
  

  Serial.print(" ");
  Serial.print(v1Filt);
  //Serial.print(pos);
  Serial.println();
  delay(1);
}

void spinArm(int dir, int pwmVal){
  if(dir == 1){ 
    // Turn one way
    roboclaw.ForwardM2(address, pwmVal);
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

  // check if angle == desired, if so trigger release
  if (pos_i == (int)sender_release_pulse_angle and (v1Filt < desired_rpm + rpmError and v1Filt > desired_rpm - rpmError)){
      noInterrupts();
      releaseBall();
      interrupts();
      ball_released = true;
    } 
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

 void releaseBall(){
  myservo.write(servoOpen);
  }
