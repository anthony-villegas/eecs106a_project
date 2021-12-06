// Pins
#define ENCA 2
#define ENCB 3

// globals
long prevT = 0;
int posPrev = 0;
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;
float desired_rpm = 0;
float sender_release_pulse_angle = 0;
float catcher_angle = 0;


void setup() {
  Serial.begin(115200);

  pinMode(ENCA,INPUT_PULLUP);
  pinMode(ENCB,INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCA),
                  readEncoder,RISING);
  
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
  
  float sender_release_pulse_angle = ((desired_omega*t_l)*180/(PI))/0.6; //0.6 to get which pulse # from 1-600
}

void loop() {

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
  float v1 = velocity1/360*60.0;
  float v2 = velocity2/360*60.0;

  // Low-pass filter (25 Hz cutoff)
  v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  v1Prev = v1;
  v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev;
  v2Prev = v2;


  Serial.print(" ");
  Serial.print(v1Filt);
  //Serial.print(pos);
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

  // Compute velocity with method 2
  //long currT = micros();
  //float deltaT = ((float) (currT - prevT_i))/1.0e6;
  //velocity_i = increment/deltaT;
  //prevT_i = currT;
}
