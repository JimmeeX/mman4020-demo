#!/usr/bin/env python

"""
Simulation of Water Sampler
"""

import rospy
import time
import sys
import numpy as np

from std_msgs.msg import Bool, Float32

SLEEP_RATE = 3

# Standard Deviation
STD_FLOW = 0.05
STD_DEPTH = 0.01
STD_TEMP = 0.05

# Expected Values
MU_FLOW = 380 / 60 # mL/sec
MU_TEMP = 20

class Simulate():
    def __init__(self):
        rospy.loginfo("Starting up Simulate")

        self.pump = False

        # Initialise Publishers
        self.depth_pub = rospy.Publisher("/arduino/depth", Float32, queue_size=1) # Water Depth (m)
        self.temp_pub = rospy.Publisher("/arduino/temp", Float32, queue_size=1) # Water Temperature (degrees C)
        self.flow_pub = rospy.Publisher("/arduino/flow", Float32, queue_size=1) # Water Flow Rate (mL/min)

        # Initialise Subscribers
        self.pump_sub = rospy.Subscriber("/arduino/pump", Bool, self.handle_pump, queue_size=1)

        self.expected_depth = -2
        self.rate = rospy.Rate(SLEEP_RATE)

        self.loop()

    def loop(self):
        """
        Generate PUMP Data
        """
        while not rospy.is_shutdown():
            # Check if pump is on
            if self.pump:
                # Simulate Flow & Temperature Readings
                flow = np.random.normal(MU_FLOW, STD_FLOW)
                temp = np.random.normal(MU_TEMP, STD_TEMP)
                self.temp_pub.publish(temp)
                self.flow_pub.publish(flow)

            self.rate.sleep()

    def handle_pump(self, msg):
        self.pump = msg.data



if __name__ == '__main__':
    rospy.init_node('simulate')
    Simulate()
    rospy.spin()