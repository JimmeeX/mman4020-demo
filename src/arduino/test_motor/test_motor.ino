#include <ros.h>
#include <std_msgs/Bool.h>

// Relay Board
const int RELAY_ENABLE_1 = 9; // Valve 1
const int RELAY_ENABLE_2 = 2; // Valve 2
const int RELAY_ENABLE_3 = 3; // Valve 3
const int RELAY_ENABLE_4 = 4; // Valve 4
const int RELAY_ENABLE_5 = 5; // Valve 5
const int RELAY_ENABLE_6 = 6; // Valve 6
const int RELAY_ENABLE_7 = 7; // Valve 7 (Purge)
const int RELAY_ENABLE_8 = 8; // Peristaltic Motor

ros::NodeHandle nh;

void handle_valve1(const std_msgs::Bool& msg) {
  if (msg.data) {digitalWrite(RELAY_ENABLE_1, LOW);}
  else {digitalWrite(RELAY_ENABLE_1, HIGH);}
}

void handle_valve2(const std_msgs::Bool& msg) {
  if (msg.data) {digitalWrite(RELAY_ENABLE_2, LOW);}
  else {digitalWrite(RELAY_ENABLE_2, HIGH);}
}

void handle_valve3(const std_msgs::Bool& msg) {
  if (msg.data) {digitalWrite(RELAY_ENABLE_3, LOW);}
  else {digitalWrite(RELAY_ENABLE_3, HIGH);}
}

void handle_valve4(const std_msgs::Bool& msg) {
  if (msg.data) {digitalWrite(RELAY_ENABLE_4, LOW);}
  else {digitalWrite(RELAY_ENABLE_4, HIGH);}
}

void handle_valve5(const std_msgs::Bool& msg) {
  if (msg.data) {digitalWrite(RELAY_ENABLE_5, LOW);}
  else {digitalWrite(RELAY_ENABLE_5, HIGH);}
}

void handle_valve6(const std_msgs::Bool& msg) {
  if (msg.data) {digitalWrite(RELAY_ENABLE_6, LOW);}
  else {digitalWrite(RELAY_ENABLE_6, HIGH);}
}

void handle_valve7(const std_msgs::Bool& msg) {
  if (msg.data) {digitalWrite(RELAY_ENABLE_7, LOW);}
  else {digitalWrite(RELAY_ENABLE_7, HIGH);}
}

void handle_pump(const std_msgs::Bool& msg) {
  if (msg.data) {digitalWrite(RELAY_ENABLE_8, LOW);}
  else {digitalWrite(RELAY_ENABLE_8, HIGH);}
}

ros::Subscriber<std_msgs::Bool> pump_sub("/arduino/pump", &handle_pump);
ros::Subscriber<std_msgs::Bool> valve1_sub("/arduino/valve1", &handle_valve1);
ros::Subscriber<std_msgs::Bool> valve2_sub("/arduino/valve2", &handle_valve2);
ros::Subscriber<std_msgs::Bool> valve3_sub("/arduino/valve3", &handle_valve3);
ros::Subscriber<std_msgs::Bool> valve4_sub("/arduino/valve4", &handle_valve4);
ros::Subscriber<std_msgs::Bool> valve5_sub("/arduino/valve5", &handle_valve5);
ros::Subscriber<std_msgs::Bool> valve6_sub("/arduino/valve6", &handle_valve6);
ros::Subscriber<std_msgs::Bool> valve7_sub("/arduino/valve7", &handle_valve7);



void setup() {
  pinMode(RELAY_ENABLE_1, OUTPUT);
  digitalWrite(RELAY_ENABLE_1,HIGH);
  
  pinMode(RELAY_ENABLE_2, OUTPUT);
  digitalWrite(RELAY_ENABLE_2,HIGH);
  
  pinMode(RELAY_ENABLE_3, OUTPUT);
  digitalWrite(RELAY_ENABLE_3,HIGH);
  
  pinMode(RELAY_ENABLE_4, OUTPUT);
  digitalWrite(RELAY_ENABLE_4,HIGH);
  
  pinMode(RELAY_ENABLE_5, OUTPUT);
  digitalWrite(RELAY_ENABLE_5,HIGH);
 
  pinMode(RELAY_ENABLE_6, OUTPUT);
  digitalWrite(RELAY_ENABLE_6,HIGH);
  
  pinMode(RELAY_ENABLE_7, OUTPUT);
  digitalWrite(RELAY_ENABLE_7,HIGH);

  pinMode(RELAY_ENABLE_8, OUTPUT);
  digitalWrite(RELAY_ENABLE_8,HIGH);


  nh.initNode();
  nh.subscribe(valve1_sub);
  nh.subscribe(valve2_sub);
  nh.subscribe(valve3_sub);
  nh.subscribe(valve4_sub);
  nh.subscribe(valve5_sub);
  nh.subscribe(valve6_sub);
  nh.subscribe(valve7_sub);
  nh.subscribe(pump_sub);
}



void loop() {
  nh.spinOnce();
}



