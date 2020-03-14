#include <ros.h>
#include <std_msgs/Float32.h>

/*
***************
PIN DEFINITIONS
***************
*/

// Flow Sensor
const int FLOW_PIN = 13;

// Relay Board
const int RELAY_ENABLE_1 = 1;
const int RELAY_ENABLE_2 = 2;
const int RELAY_ENABLE_3 = 3;
const int RELAY_ENABLE_4 = 4;
const int RELAY_ENABLE_5 = 5;
const int RELAY_ENABLE_6 = 6;
const int RELAY_ENABLE_7 = 7;
const int RELAY_ENABLE_8 = 8;

// Relay to Motors/Actuators


/*
****************
GLOBAL VARIABLES
****************
*/
ros::NodeHandle nh;
std_msgs::Float32 flow_msg;

/*
*****************************
DEFINE PUBLISHERS/SUBSCRIBERS
*****************************
*/
ros::Publisher pub_flow("/arduino/flow", &flow_msg);

void setup()
{
  // Setup Pins
  pinMode(FLOW_PIN, INPUT);
  
  // Setup Publishers/Subscribers
  nh.initNode();
  
  nh.advertise(pub_flow);
}

void loop()
{
  // readFlow();
  nh.spinOnce();
}

