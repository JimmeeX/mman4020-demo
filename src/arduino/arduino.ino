#include <ros.h>
#include <std_msgs/Float32.h>

/***************
PIN DEFINITIONS
***************/
// Relay Board
const int RELAY_ENABLE_1 = 1; // Valve 1
const int RELAY_ENABLE_2 = 2; // Valve 2
const int RELAY_ENABLE_3 = 3; // Valve 3
const int RELAY_ENABLE_4 = 4; // Valve 4
const int RELAY_ENABLE_5 = 5; // Valve 5
const int RELAY_ENABLE_6 = 6; // Valve 6
const int RELAY_ENABLE_7 = 7; // Valve 7
const int RELAY_ENABLE_8 = 8; // Peristaltic Motor

// Flow Sensor
const int FLOW_PIN = 12; // Flow Sensor

/****************
GLOBAL VARIABLES
****************/
ros::NodeHandle nh;
std_msgs::Float32 flow_msg;

/****************************
DEFINE PUBLISHERS/SUBSCRIBERS
****************************/
ros::Publisher pub_flow("/arduino/flow", &flow_msg);

void setup() {
  // Setup Pin Definitions
  pinMode(FLOW_PIN, INPUT);
  pinMode(RELAY_ENABLE_1, OUTPUT);
  pinMode(RELAY_ENABLE_2, OUTPUT);
  pinMode(RELAY_ENABLE_3, OUTPUT);
  pinMode(RELAY_ENABLE_4, OUTPUT);
  pinMode(RELAY_ENABLE_5, OUTPUT);
  pinMode(RELAY_ENABLE_6, OUTPUT);
  pinMode(RELAY_ENABLE_7, OUTPUT);
  pinMode(RELAY_ENABLE_8, OUTPUT);
  
  // Setup Publishers/Subscribers
  nh.initNode();
  
  nh.advertise(pub_flow);
}

void loop() {
  nh.spinOnce();
}
