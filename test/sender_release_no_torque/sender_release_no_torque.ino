// Pins
#define ENCA 2
#define ENCB 3
#define INDUCT 20
#define servoPin 9

#include <Servo.h>

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
float sender_release_pulse_angle = 0;

//pid
float eintegral = 0;

// open and closed angles of servo
int servoOpen = 150;
int servoClosed = 100;

// position of inductor in encoder counts
volatile int inductor_zero = 0;

// check if ball released
bool ball_released = false;

// counts of encoder
int encoder_counts = 600;


void setup() { 
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

  // want to release 45 from the horizontal which is 0 + 90 + 45
  int desired_release_angle = 135;

  // if we go at this rpm
  int desired_rpm = 5;
  int desired_rps = desired_rpm * 60;
  
  // and the error time is .1 ms
  float e_t = 0.1;

  // then the distance traveled for release is
  float e_distance = desired_rps * e_t;

  // so we actually want to release at this angle
  float actual_release_angle = desired_release_angle - e_distance;

  // converting this to encoder counts
  sender_release_pulse_angle = actual_release_angle/.6; 

  myservo.write(servoClosed);
  //giver servo time to close
  delay(5000);
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

 
  

  
  //Serial.print(v1Filt);
  Serial.print("Pos: ");
  Serial.print(pos);
  Serial.println();
  Serial.print("Pulse Angle: ");
  Serial.print(sender_release_pulse_angle);
  Serial.println();
  delay(1);
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
  if ((pos_i % encoder_counts) == ((int)sender_release_pulse_angle)){
      noInterrupts();
      releaseBall();
      ball_released = true;
      interrupts();
    } 
}

void senseInductive(){
  int pos = 0;
  noInterrupts(); // disable interrupts temporarily while reading
  pos = pos_i;
  interrupts(); // turn interrupts back on
     
  if (pos_induct = -999){
    pos_induct = pos % encoder_counts;
    sender_release_pulse_angle = pos_induct + sender_release_pulse_angle;
    }  
 }

 void releaseBall(){
  myservo.write(servoOpen);
  }
