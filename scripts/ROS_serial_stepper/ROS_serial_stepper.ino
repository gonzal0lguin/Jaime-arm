#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <AccelStepper.h>
#include <ros.h>
#include <std_msgs/Float64.h>

#define stepPin 9
#define dirPin 8
#define enablePin 10
#define servoPin 7

ros::NodeHandle  nh;

Servo servo;
AccelStepper nema = AccelStepper(AccelStepper::DRIVER, stepPin, dirPin);

void servo_cb( const std_msgs::Float64& cmd_msg){
  servo.write(cmd_msg.data); //set servo angle, should be from 0-180  
  //digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}


void stepper_enable(){
  digitalWrite(enablePin, LOW);
}

void stepper_disable(){
  digitalWrite(enablePin, HIGH);
}


void moveStepper(int p, float speed, float accel){
  nema.setMaxSpeed(speed);
  nema.setAcceleration(accel);
  stepper_enable();
  nema.moveTo(p);

  while (nema.distanceToGo() != 0){
    nema.run();
    }
    
  stepper_disable();
}

void speedStepper(float speed){
  //stepper_enable();
  nema.setSpeed(speed);
}

void stepper_cb(const std_msgs::Float64& cmd_msg){
  stepper_enable();
  nema.setSpeed(cmd_msg.data);
  
  if (nema.currentPosition() > 2200){
    nema.setSpeed(0);
    nema.setCurrentPosition(2199);
    stepper_disable();
    }

   else if (nema.currentPosition() < 0){
    nema.setSpeed(0);
    nema.setCurrentPosition(0);
    stepper_disable();
    }
  }


ros::Subscriber<std_msgs::Float64> subservo("/servo", servo_cb);
ros::Subscriber<std_msgs::Float64> substepper("/stepper", stepper_cb);

void setup(){
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(servoPin, OUTPUT);
  stepper_disable();
  nh.initNode();
  nh.subscribe(subservo);
  nh.subscribe(substepper);
  nema.setMaxSpeed(600);
  servo.attach(servoPin);
}

void loop(){
  nh.spinOnce();
  delay(1);
  nema.runSpeed();
}
