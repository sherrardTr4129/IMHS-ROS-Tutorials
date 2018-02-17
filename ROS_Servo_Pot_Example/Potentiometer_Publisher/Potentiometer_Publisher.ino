/*
 * Author: Trevor Sherrard
 * Date: 02/17/2018
 * Project: Intelligent Material Handling System
 */

/***********************************************
 * Includes 
 ***********************************************
 */
#include <ros.h>
#include <std_msgs/Int32.h>

/***********************************************
 * Defines
 ***********************************************
 */
#define POT_PIN   A0
#define MAX_IN    1023
#define MAX_OUT   180
#define MIN_IN    0
#define MIN_OUT   0

/***********************************************
 * Variables 
 ***********************************************
 */
long potentiometer_val = 0;
long mapped_val = 0;

ros::NodeHandle  nh;
std_msgs::Int32 pot_val_mapped_msg;
ros::Publisher ServoController("ServoController", &pot_val_mapped_msg);

void setup() {
  pinMode(POT_PIN, INPUT);
  
  nh.initNode();
  nh.advertise(ServoController);  
}

void loop() {
  potentiometer_val = analogRead(POT_PIN);
  mapped_val = map(potentiometer_val, MIN_IN, MAX_IN, MIN_OUT, MAX_OUT);
  mapped_val = constrain(mapped_val, MIN_OUT, MAX_OUT);

  pot_val_mapped_msg.data = mapped_val;
  ServoController.publish(&pot_val_mapped_msg);
  
  nh.spinOnce();
  delay(50);
}
