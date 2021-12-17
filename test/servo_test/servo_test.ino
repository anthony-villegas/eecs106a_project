/* Servo motor with Arduino example code. Position and sweep. More info: https://www.makerguides.com/ */
// Include the servo library:
#include <Servo.h>
// Create a new servo object:
Servo myservo;
// Define the servo pin:
#define servoPin 9
// Create a variable to store the servo position:
int angleClosed = 80;
int angleOpen = 150;
void setup() {
  // Attach the Servo variable to a pin:
  myservo.attach(servoPin);
  Serial.begin(9600);
}
void loop() {
  //Serial.println("Input angle for servo"); //Prompt User for Input
  //while (Serial.available() == 0) {
    // Wait for User to Input Data
  //}
  //angle = Serial.parseInt(); //Read the data the user has input
  //Serial.println(angle);
  //myservo.write(angle);
  //delay(1000);
  //myservo.write(angleOpen);
  myservo.write(angleClosed);
  
}
