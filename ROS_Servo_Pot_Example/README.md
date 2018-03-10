# Potentiometer Control Of Servo Via ROS

This tutorial walks readers through the setup of hardware and software necessary for implementing control of a hobby servo with a potentiometer via ROS topics.

# Hardware Setup
![Hardware Overview](images/Hardware_Setup.jpg)

This demo will make use of a Teensy3.2 and a Arduino UNO as the main micro-controllers. These two micro-controllers connect to a ROS capable laptop computer via a USB connection. One micro-controller reads in a value from a potentiometer, while the other drives a servo using PWM control. See the figure below for a schematic view of this setup.

![schematic](images/schematic.png)

Please note that the above schematic does not depict the connection of the micro-controllers to the ROS enabled laptop.


# Software Setup

This tutorial assumes the user has installed the rosserial meta-package as well as the ROS arduino libraries. See this [page](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup) for tutorials on getting these installed if you have not already.

In this example, a ROS node running on the Teensy publishes data read from the potentiometer over the Int32 topic. A ROS node running on the Arduino UNO subscribes to this data and moves the servo accordingly. On the laptop, two instances of the rosserial bridge instance run to control serial communication with each device. These nodes functionally act as a software bridge between the ROS nodes on the micro-controllers and the rest of the system.

Let's walk through some of the code of the potentiometer publisher node.

```cpp
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

```

Here we can see define constants and variables for potentiometer values. Notice the explicit delcaration of an Int32 message instance. In the last line, we instantiate the ros publisher object.

# Demo Video Link

See this youtube [video](https://youtu.be/ZQzFy3OdAiA) for a demo of the tutorial result.
