#include <AFMotor.h>


#define lefts A3    // Change to IR sensor Connected pin number
#define rights A4  // Change to IR sensor Connected pin number


AF_DCMotor motor1(1, MOTOR12_8KHZ);   // Here Motor 1 is connected to M1 Pin
AF_DCMotor motor2(4, MOTOR12_8KHZ);   // Here Motor 2 is connected to M4 Pin


void setup() {


  pinMode(lefts,INPUT);
  pinMode(rights,INPUT);

  Serial.begin(9600);
  
}

void loop(){
  //printing values of the sensors to the serial monitor
  
  Serial.println(analogRead(lefts));
  Serial.println(analogRead(rights));
  
  //line detected by both
  if(analogRead(lefts)<=400 && analogRead(rights)<=400){
    //FORWARD
    motor1.run(FORWARD);
    motor1.setSpeed(100);
    motor2.run(FORWARD);
    motor2.setSpeed(100);
    

  }
  //line detected by left sensor
  else if(analogRead(lefts)<=400 && !analogRead(rights)<=400){
    //turn left
    motor1.run(FORWARD);
    motor1.setSpeed(100);
    motor2.run(BACKWARD);
    motor2.setSpeed(90);


  }
  //line detected by right sensor
  else if(!analogRead(lefts)<=400 && analogRead(rights)<=400){;
    //turn right
    motor1.run(BACKWARD);
    motor1.setSpeed(90);
    motor2.run(FORWARD);
    motor2.setSpeed(100);


  }
  //line detected by none
  else if(!analogRead(lefts)<=400 && !analogRead(rights)<=400){
    //STOP
    motor1.run(RELEASE);
    motor2.run(RELEASE);

  }
  
}
