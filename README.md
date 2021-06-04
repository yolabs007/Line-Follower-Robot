# Line-Follower-Robot


* Check the sensors with IR sensor test Code first, then go with Line follower Code.

```C++

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

```


* Select the Digital write code for Line Follower Robot for better accuracy.

# Mounting 

* Place the IR sensors very near to ground.
* Do not give more space in between two sensors.
