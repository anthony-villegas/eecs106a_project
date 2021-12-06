int limitswitch=4;
int state= LOW;
int value;
void setup()
{
Serial.begin(9600);
pinMode(limitswitch,INPUT);
 
}
void loop()
{
value = digitalRead(limitswitch);
if(value!=state)
{
  state=value;
  Serial.println("sensor value =");
  if (state==0)
  {
    Serial.println("target detected");
  }
    
   else{
     Serial.println("No target detected");
    }
 }
 delay(300);
}
