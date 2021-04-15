void setup() {
  pinMode(7,INPUT);    // Change this pin number to IR Sensor Connected pin
  Serial.begin(9600);
  pinMode(13,OUTPUT);   //Arduino In-Built LED pin
}

void loop() {
  
Serial.print("IRSensorip  ");
Serial.println(digitalRead(7));

if(digitalRead(7)==0)      // if IR sensor detects the bright color
{
  digitalWrite(13,HIGH);    // In- Built LED will Glow
  }
 else{
    digitalWrite(13,LOW);
    }

}
