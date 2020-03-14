// Test SG90 Servo Motor

#include <ros.h>
#include <std_msgs/Int16.h>

#include <Servo.h>

ros::NodeHandle nh;

Servo servo;

const int servoPin = 11;

void handleServo(const std_msgs::Int16& servo_msg) {
  servo.write(servo_msg.data);
  delay(15);
}

ros::Subscriber<std_msgs::Int16> sub_servo("/arduino/servo", &handleServo);

void setup() {
  servo.attach(servoPin);
  servo.write(0); // Initial Angle?
  
  nh.initNode();
  nh.subscribe(sub_servo);
}

void loop() {
  nh.spinOnce();
  delay(1);
}

