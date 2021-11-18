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

ros::NodeHandle  nh;

Servo servo;
AccelStepper nema = AccelStepper(AccelStepper::DRIVER, stepPin, dirPin);

void servo_cb( const std_msgs::Float64& cmd_msg){
  servo.write(cmd_msg.data); //set servo angle, should be from 0-180  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}


void moveStepper(int p, float speed, float accel){
  nema.setMaxSpeed(speed);
  nema.setAcceleration(accel);
  //nema.disableOutputs();
  nema.moveTo(p);

  while (nema.distanceToGo() != 0){
    nema.run();
    }
    
  //nema.enableOutputs();
  }

void stepper_cb(const std_msgs::Float64& cmd_msg){
  moveStepper(cmd_msg.data, 600, 600);
  }


//ros::Subscriber<std_msgs::UInt16> sub("servo", servo_cb);
ros::Subscriber<std_msgs::Float64> sub("stepper", stepper_cb);

void setup(){
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  
  //servo.attach(9); //attach it to pin 9
}

void loop(){
  nh.spinOnce();
  delay(1);
}
