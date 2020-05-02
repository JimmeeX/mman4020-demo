#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include "solenoid.h"
#include "pump.h"

/*
***************
PIN DEFINITIONS
***************
*/

// Flow Sensor
const int FLOW_PIN = 13;

// Relay Board
const int RELAY_ENABLE_1 = 9; // Valve 1
const int RELAY_ENABLE_2 = 2; // Valve 2
const int RELAY_ENABLE_3 = 3; // Valve 3
const int RELAY_ENABLE_4 = 4; // Valve 4
const int RELAY_ENABLE_5 = 5; // Valve 5
const int RELAY_ENABLE_6 = 6; // Valve 6
const int RELAY_ENABLE_7 = 7; // Valve 7 (Purge)
const int RELAY_ENABLE_8 = 8; // Peristaltic Motor


/*
****************
GLOBAL VARIABLES
****************
*/
Solenoid valve1 = Solenoid(RELAY_ENABLE_1);
Solenoid valve2 = Solenoid(RELAY_ENABLE_2);
Solenoid valve3 = Solenoid(RELAY_ENABLE_3);
Solenoid valve4 = Solenoid(RELAY_ENABLE_4);
Solenoid valve5 = Solenoid(RELAY_ENABLE_5);
Solenoid valve6 = Solenoid(RELAY_ENABLE_6);
Solenoid valve7 = Solenoid(RELAY_ENABLE_7);

Pump pump = Pump(RELAY_ENABLE_8);

ros::NodeHandle nh;
std_msgs::Float32 flow_msg;

/*
****************
DEFINE CALLBACKS
****************
*/
void handle_valve1(const std_msgs::Bool& msg){
  valve1.set(msg.data);
}

void handle_valve2(const std_msgs::Bool& msg){
  valve2.set(msg.data);
}

void handle_valve3(const std_msgs::Bool& msg){
  valve3.set(msg.data);
}

void handle_valve4(const std_msgs::Bool& msg){
  valve4.set(msg.data);
}

void handle_valve5(const std_msgs::Bool& msg){
  valve5.set(msg.data);
}

void handle_valve6(const std_msgs::Bool& msg){
  valve6.set(msg.data);
}

void handle_valve7(const std_msgs::Bool& msg){
  valve7.set(msg.data);
}

void handle_pump(const std_msgs::Bool& msg){
  pump.set(msg.data);
}

/*
*****************************
DEFINE PUBLISHERS/SUBSCRIBERS
*****************************
*/
//ros::Publisher pub_flow("/arduino/test", &flow_msg);

//ros::Subscriber<std_msgs::Bool> valve1_sub("/arduino/valve1", &handle_valve1);
//ros::Subscriber<std_msgs::Bool> valve2_sub("/arduino/valve2", &handle_valve2);
//ros::Subscriber<std_msgs::Bool> valve3_sub("/arduino/valve3", &handle_valve3);
//ros::Subscriber<std_msgs::Bool> valve4_sub("/arduino/valve4", &handle_valve4);
//ros::Subscriber<std_msgs::Bool> valve5_sub("/arduino/valve5", &handle_valve5);
//ros::Subscriber<std_msgs::Bool> valve6_sub("/arduino/valve6", &handle_valve6);
//ros::Subscriber<std_msgs::Bool> valve7_sub("/arduino/valve7", &handle_valve7);
ros::Subscriber<std_msgs::Bool> pump_sub("/arduino/pump", &handle_pump);


void setup()
{
  
  // Setup Pins
//  pinMode(RELAY_ENABLE_1, OUTPUT);
//  pinMode(RELAY_ENABLE_2, OUTPUT);
//  pinMode(RELAY_ENABLE_3, OUTPUT);
//  pinMode(RELAY_ENABLE_4, OUTPUT);
//  pinMode(RELAY_ENABLE_5, OUTPUT);
//  pinMode(RELAY_ENABLE_6, OUTPUT);
//  pinMode(RELAY_ENABLE_7, OUTPUT);
  pinMode(RELAY_ENABLE_8, OUTPUT);
  
  digitalWrite(RELAY_ENABLE_8, HIGH);


  // Setup Publishers/Subscribers
  nh.initNode();

//  nh.advertise(pub_flow);

//  nh.subscribe(valve1_sub);
//  nh.subscribe(valve2_sub);
//  nh.subscribe(valve3_sub);
//  nh.subscribe(valve4_sub);
//  nh.subscribe(valve5_sub);
//  nh.subscribe(valve6_sub);
//  nh.subscribe(valve7_sub);
  nh.subscribe(pump_sub);
}

void loop()
{
  nh.spinOnce();
}

