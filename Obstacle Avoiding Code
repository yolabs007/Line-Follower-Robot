#include "AFMotor.h"

#define echopin A4 // echo pin
#define trigpin A5 // Trigger pin

 

AF_DCMotor motor1(1, MOTOR12_64KHZ); // create motor object, 64KHz pwm
AF_DCMotor motor2(4, MOTOR12_64KHZ); // create motor object, 64KHz pwm



int distance_L, distance_F, distance_R;
long distance;

int set = 20;
 
void setup() {
  Serial.begin(9600);           // Initialize serial port
  Serial.println("Start");

  pinMode (trigpin, OUTPUT);
  pinMode (echopin, INPUT );
  
  motor1.setSpeed(255);          // set the motor speed to 0-255
  motor2.setSpeed(255);

}

void loop() {
 distance_F = data();
 Serial.print("S=");
 Serial.println(distance_F);
  if (distance_F > set){
   Serial.println("Forward");
  motor1.run(FORWARD);         // turn it on going forward
  motor2.run(FORWARD); 
    }
    else{hc_sr4();}
}


long data(){
  digitalWrite(trigpin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigpin, HIGH);
  delayMicroseconds(10);
  distance = pulseIn (echopin, HIGH);
  return distance / 29 / 2;
}


void compareDistance(){
  if (distance_L > distance_R){
  motor1.run(BACKWARD);   // turn it on going left
  motor2.run(FORWARD); 
  delay(350);
  }
  else if (distance_R > distance_L){
  motor1.run(FORWARD);  // the other right
  motor2.run(BACKWARD); 
  delay(350);
  }
  else{
  motor1.run(BACKWARD);  // the other way
  motor2.run(BACKWARD);
  delay(300);
  motor1.run(BACKWARD);   // turn it on going left
  motor2.run(FORWARD); 
  delay(500);
  }
}

void hc_sr4(){
    Serial.println("Stop");
    motor1.run(RELEASE);         // stopped
    motor2.run(RELEASE);

    distance_R = data();
    delay(100);
    distance_L = data();
    delay(100);
    compareDistance();
}
