// Tested HC-SR04

#include <ros.h>
#include <std_msgs/Float32.h>

std_msgs::Float32 ultra_msg;
ros::Publisher pub_ultra("/arduino/ultrasonic", &ultra_msg);
ros::NodeHandle nh;

const int trigPin = 12;
const int echoPin = 13;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  nh.initNode();
  nh.advertise(pub_ultra);
}

long duration;
long distance;

void loop() {
  readUltra();
  nh.spinOnce();
}

void readUltra() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  duration = pulseIn(echoPin, HIGH);
  
  distance = duration * 0.034 / 2;
  
  ultra_msg.data = distance;
  pub_ultra.publish(&ultra_msg);
}
