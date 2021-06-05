# Line-Follower-Robot


#### `Testing of Arduino`

* Test the Arduino Board by using blinking an In-Built LED (Below code) 

```C++
/*
  This Code is written by Rahul Sharma for Yolabs. 
This is the  simplest code possible to blink in build LED  
Turns inbuild LED on and off at diff frequency to chk your arduino IDE, Arduino and cable is working
Note: please check the port in case you have error while uploadig 
 www.yolabs.in - 2020
  
*/

// the setup function runs once when you press reset or power the board

void setup()
{
  pinMode(13,OUTPUT);
  Serial.begin(9600);
}

void loop()
{
    digitalWrite(13, HIGH);
    
    Serial.println("I am High");
    delay(3000); // Wait for 1000 millisecond(s)
    digitalWrite(13, LOW);
    Serial.println("I am Low");
    delay(3000);
 
}


```


* Check the IR sensors with IR sensor test Code (Below one) first, then go with Line follower Code.

![IR Sensor](https://5.imimg.com/data5/WA/GS/MY-5726208/delta-plc-repair-service-500x500.jpg)

#### `Connections`


IR sensor | Arduino
------------ | -------------
OUT | Pin Number 7
VCC | 5V
GND | GND



#### `IR Sensor Testing code`

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


* Select the Digital write code (Below) for Line Follower Robot for better accuracy.


![IR Sensor](https://content.instructables.com/ORIG/FFZ/OG6Q/JGGTIBUY/FFZOG6QJGGTIBUY.jpg?auto=webp)


#### `Connections`


IR sensors | Arduino Shield
------------ | -------------
Left Side Sensor - OUT | Pin Number A3
Right Side Sensor - OUT | Pin Number A4
GND | GND
VCC | 5V 



#### `Line follower Robot Code`


```C++

#include <AFMotor.h>

// Change these below Pins to connected pins of IR sensor

#define lefts A3 
#define rights A4 


AF_DCMotor motor1(1, MOTOR12_8KHZ);   // Here right side motor is connected to M1
AF_DCMotor motor2(4, MOTOR12_8KHZ);   // Here left side motor is connected to M4


void setup() {


  pinMode(lefts,INPUT);
  pinMode(rights,INPUT);

  Serial.begin(9600);
  
}

void loop(){
  //printing values of the sensors to the serial monitor
  Serial.println(digitalRead(lefts));
  Serial.println(digitalRead(rights));
  //line detected by both
  if(digitalRead(lefts)==0 && digitalRead(rights)==0){
    //FORWARD
    motor1.run(FORWARD);
    motor1.setSpeed(125);
    motor2.run(FORWARD);
    motor2.setSpeed(125);

  }
  //line detected by left sensor
  else if(digitalRead(lefts)==0 && digitalRead(rights)==1){
    //turn left
    motor1.run(FORWARD);
    motor1.setSpeed(125);
    motor2.run(BACKWARD);
    motor2.setSpeed(100);


  }
  //line detected by right sensor
  else if(digitalRead(lefts)==1 && digitalRead(rights)==0){;
    //turn left
    motor1.run(BACKWARD);
    motor1.setSpeed(100);
    motor2.run(FORWARD);
    motor2.setSpeed(125);


  }
  //line detected by none
  else if(digitalRead(lefts)==1 && digitalRead(rights)==1){
    //stop
    motor1.run(RELEASE);
    motor2.run(RELEASE);

  }
  
}

```

# Mounting 

* Place the IR sensors very near to ground.
* Do not give more space in-between two sensors.
* Don't use sharp turns (90 degree turns), it's very difficult for vehicle to turn.
