//const int Pin=20;
#define INDUCT 20
void setup() {
    pinMode(INDUCT, INPUT);
    attachInterrupt(digitalPinToInterrupt(INDUCT),
                  senseInductive,RISING);
    Serial.begin(9600);
}
 
void loop() {
    //int sensorValue = digitalRead(Pin);
    //if(sensorValue==HIGH){ 
      //  Serial.println("no Object");
        //delay(500);
    //}
    //else{
      //  Serial.println("Object Detected");
        //delay(500);
    //}
}

void senseInductive(){
  Serial.println("hi!");
  }
