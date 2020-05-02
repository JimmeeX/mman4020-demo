#!/usr/bin/env python

"""
Simulation of Water Sampler
"""

import rospy
import time
import sys
import numpy as np

from std_msgs.msg import Bool, Float32, Float64

SLEEP_RATE = 3

# Standard Deviation
STD_FLOW = 0.1
STD_DEPTH = 0.01
STD_TEMP = 0.05

# Expected Values
# MU_FLOW = 380 / 60 # mL/sec
MU_FLOW = 410.0 / 60.0
MU_FLOW_AIR = 120.0 / 60.0
MU_TEMP = 20.0

TUBE_LENGTH = 7.5 # m

class Simulate():
    def __init__(self):
        rospy.loginfo("Starting up Simulate")

        self.pump = False
        self.valve1 = False
        self.valve2 = False
        self.valve3 = False

        # Initialise Publishers
        self.depth_pub = rospy.Publisher("/arduino/depth", Float32, queue_size=1) # Water Depth (m)
        self.temp_pub = rospy.Publisher("/arduino/temp", Float32, queue_size=1) # Water Temperature (degrees C)
        self.flow_pub = rospy.Publisher("/arduino/flow", Float32, queue_size=1) # Water Flow Rate (mL/min)

        self.valve1_sub = rospy.Subscriber("/arduino/valve1", Bool, self.handle_valve, (1), queue_size=1) # Jar 1
        self.valve2_sub = rospy.Subscriber("/arduino/valve2", Bool, self.handle_valve, (2), queue_size=1) # Jar 2
        self.valve3_sub = rospy.Subscriber("/arduino/valve3", Bool, self.handle_valve, (3), queue_size=1) # Jar 3

        # Initialise Subscribers
        self.pump_sub = rospy.Subscriber("/arduino/pump", Bool, self.handle_pump, queue_size=1)
        self.alt_sub = rospy.Subscriber("/mavros/global_position/rel_alt", Float64, self.handle_alt, queue_size=1) # Requires SITL / Actual Drone

        self.expected_depth = -2
        self.rate = rospy.Rate(SLEEP_RATE)

        self.loop()

    def loop(self):
        """
        Generate PUMP Data
        """
        counter = 0
        while not rospy.is_shutdown():
            # Check if pump is on
            if self.pump:
                if self.valve1:
                    flow = np.random.normal(MU_FLOW, STD_FLOW)
                    temp = np.random.normal(20.0, STD_TEMP)
                    # print(flow)

                elif self.valve2:
                    counter += 1
                    if counter < 20 * SLEEP_RATE:
                        flow = np.random.normal(MU_FLOW_AIR, STD_FLOW)
                        temp = np.random.normal(21.0, STD_TEMP)
                    else:
                        flow = np.random.normal(MU_FLOW, STD_FLOW)
                        temp = np.random.normal(21.0, STD_TEMP)

                elif self.valve3:
                    flow = np.random.normal(MU_FLOW, STD_FLOW)
                    temp = np.random.normal(25.0, STD_TEMP)

                else:
                    # Simulate Flow & Temperature Readings
                    flow = np.random.normal(MU_FLOW, STD_FLOW)
                    temp = np.random.normal(MU_TEMP, STD_TEMP)
                self.temp_pub.publish(temp)
                self.flow_pub.publish(flow)

            depth = np.random.normal(-2.5, STD_DEPTH)
            self.depth_pub.publish(depth)

            self.rate.sleep()

    def handle_pump(self, msg):
        self.pump = msg.data

    def handle_valve(self, msg, id):
        if id == 1: self.valve1 = msg.data
        elif id == 2: self.valve2 = msg.data
        elif id == 3: self.valve3 = msg.data

    def handle_alt(self, msg):
        """Naive approximation of water depth by drone altitude - length of dangling tube"""
        depth = np.random.normal(msg.data - TUBE_LENGTH, STD_DEPTH)
        self.depth_pub.publish(depth)



if __name__ == '__main__':
    rospy.init_node('simulate')
    Simulate()
    rospy.spin()