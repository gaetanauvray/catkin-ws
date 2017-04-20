/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <ros_control_boilerplate/UInt16Arr.h>

#include <Stepper.h>

ros::NodeHandle nh;

//servo param
Servo servo;
int servoPin = 3;

//stepper param
const int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution
float curPos = 0;
float newPos = 0;
float steps = 0;

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 8, 10, 9, 11);


//chatterpublisher
ros_control_boilerplate::UInt16Arr pos_msg;
ros::Publisher chatter_pub("/arduino/pos_message", &pos_msg);



void servo_cb( const ros_control_boilerplate::UInt16Arr& cmd_msg){
  servo.write(cmd_msg.data[1]); //set servo angle, should be from 0-180

//  newPos = (float)cmd_msg.data[0];
  
//  if(abs(newPos-curPos)<abs(-curPos-360+newPos)){
//    steps = (newPos -curPos) / 360 * stepsPerRevolution;}
//  else{
//    steps = ((newPos -curPos -360) / 360 * stepsPerRevolution);
//  }
  
//  myStepper.step((int)steps);
  
  //curPos = newPos;
  pos_msg = cmd_msg;
  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}


ros::Subscriber<ros_control_boilerplate::UInt16Arr> sub("/mfbot/cmd_message", servo_cb);

void setup(){
  pinMode(servoPin, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter_pub);
  
  servo.attach(servoPin); //attach it to pin 9
  // intitialise stepper at speed:
  myStepper.setSpeed(2.9);
  
  
  pos_msg.data[0] = 0;
  pos_msg.data[1] = 90;
  
}

void loop(){
  
  nh.spinOnce();
  
  chatter_pub.publish( &pos_msg);
  
  delay(50);
}
