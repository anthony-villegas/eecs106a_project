//See BareMinimum example for a list of library functions

//Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include "RoboClaw.h"

//See limitations of Arduino SoftwareSerial
SoftwareSerial serial(19,18);	
RoboClaw roboclaw(&serial,10000);

#define address 0x80

void setup() {
  //Open roboclaw serial ports
  roboclaw.begin(38400);
  
 
}

void loop() {
  roboclaw.BackwardM2(address, 0);
  
  
}
