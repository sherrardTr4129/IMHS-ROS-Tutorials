/*
 * Author: Trevor Sherrard
 * Date: 02/17/2018
 * Project: Intelligent Material Handling System
 */

/***********************************************
 * Defines
 ***********************************************
 */
 #define SERVO_PIN 9

/***********************************************
 * Includes 
 ***********************************************
 */
#include <ros.h>
#include <std_msgs/Int32.h>
#include <Servo.h>

/***********************************************
 * Function Prototypes 
 ***********************************************
 */
void ServoUpdateCallback(const std_msgs::Int32& mapped_val);

/***********************************************
 * Variables 
 ***********************************************
 */
long servo_val = 0;

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int32> subscriber("ServoController", &ServoUpdateCallback);

Servo Servo_1;

void setup() {
  Servo_1.attach(SERVO_PIN);
  nh.initNode();
  nh.subscribe(subscriber);
}

void loop() {
  nh.spinOnce();
  delay(50);  
}

void ServoUpdateCallback(const std_msgs::Int32& mapped_val)
{
  Servo_1.write(mapped_val.data);
}
