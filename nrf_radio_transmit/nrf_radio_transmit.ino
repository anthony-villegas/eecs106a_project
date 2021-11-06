//This code allows us to TRANSMIT using the nrf24 module

//Include Libraries
//Libraries can be downladed here: https://github.com/nRF24/RF24
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//create an RF24 object
//42 and 40 are the digital pins we connect to on the mega
RF24 radio(42, 40);  // CE, CSN

//address through which two modules communicate.
const byte address[6] = "00001";

void setup()
{
  radio.begin();
  
  //set the address
  radio.openWritingPipe(address);
  
  //Set module as transmitter
  radio.stopListening();
}
void loop()
{
  //Send message to receiver
  const char text[] = "Hello World";
  //returns boolean on whether message recieved, should delay launch sequence if message not recieved
  radio.write(&text, sizeof(text));
  
  delay(1000);
}
