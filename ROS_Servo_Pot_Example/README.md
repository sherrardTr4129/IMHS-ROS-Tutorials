# Potentiometer Control Of Servo Via ROS

This tutorial walks readers through the setup of hardware and software necessary for implementing control of a hobby servo with a potentiometer via ROS topics.

# Hardware Setup
![Hardware Overview](images/Hardware_Setup.jpg)

This demo will make use of a Teensy3.2 and a Arduino UNO as the main micro-controllers. These two micro-controllers connect to a ROS capable laptop computer via a USB connection. One micro-controller reads in a value from a potentiometer, while the other drives a servo using PWM control. See the figure below for a schematic view of this setup. Note that two Arduino UNOs are used in the schematic, but in implementation any micro-controller supporting ROS serial will work.


# Software Setup


# Demo Video Link

See this youtube [video](https://youtu.be/ZQzFy3OdAiA) for a demo of the tutorial result.
