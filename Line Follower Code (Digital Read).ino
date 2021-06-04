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
