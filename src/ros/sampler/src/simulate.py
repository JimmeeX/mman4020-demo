#!/usr/bin/env python

"""
Simulation of Water Sampler
"""

import rospy
import time

from std_msgs.msg import Bool, Float32

class Simulate():
    def __init__(self):
        rospy.loginfo("Starting up Main")

        # Initialise Publishers
        self.depth_pub = rospy.Publisher("/arduino/depth", Float32) # Water Depth (m)
        self.temp_pub = rospy.Publisher("/arduino/temperature", Float32) # Water Temperature (degrees C)
        self.flow_pub = rospy.Publisher("/arduino/flow_rate", Float32) # Water Flow Rate (mL/min)

        # Simulation Parameters
        self.noise_baro = 0.1 # Error (%)
        self.noise_flow = 0.1 # Error (%)
        self.dt = 0.050       # Rate of data (s)

    def loop(self):
        """
        TODO: Generate Data
        """
        # while True:
        pass
        #     time.sleep(self.dt)



if __name__ == '__main__':
    rospy.init_node('main')
    Simulate()
    rospy.spin()