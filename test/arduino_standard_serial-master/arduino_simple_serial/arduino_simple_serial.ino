#include <SoftwareSerial.h>

SoftwareSerial serial(10, 11);

void setup() {
  serial.begin(38400);
}

void loop() {
  //Turn motor1 forward at half speed
  serial.write(94);
  delay(2000);
  //Stop motor1
  serial.write(64);
  delay(2000);
  //Turn motor1 backwards at half speed
  serial.write(32);
  delay(2000);
  //Stop motor1
  serial.write(64);
  delay(2000);

  //Turn motor2 forward at half speed
  serial.write(223);
  delay(2000);
  //Stop motor2 
  serial.write(192);
  delay(2000);
  //Turn motor2 backwards at half speed
  serial.write(160);
  delay(2000);
  //Stop motor2
  serial.write(192);
  delay(2000);

}
