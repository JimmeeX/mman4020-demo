#!/usr/bin/env python

import roslib
import rospy
from actionlib import SimpleActionServer

from std_msgs.msg import Bool, Float32
from sampler.msg import (
    PurgeAction,
    PurgeFeedback,
    PurgeResult,
    SampleAction,
    SampleFeedback,
    SampleResult,
    SetPumpAction,
    SetPumpResult,
    SetValveAction,
    SetValveResult,
    StopAction,
    StopResult
)

from Sampler import Sampler

import time

SLEEP_RATE = 3 # Hz (ie, 3 times/second)

PURGE_DURATION = 30 # Seconds
NUM_JARS = 6
NUM_VALVES = 7
JAR_CAPACITIES = [90] * NUM_JARS # Max Volume (mL)

class Main():
    def __init__(self):
        rospy.loginfo("Starting up Main")

        self.rate = rospy.Rate(SLEEP_RATE)

        """Initialise Water Sampler Class"""
        self.sampler = Sampler(JAR_CAPACITIES)

        """Initialise Publishers"""
        # Arduino Control Panel
        self.valve1_pub = rospy.Publisher("/arduino/valve1", Bool, queue_size=1) # Jar 1
        self.valve2_pub = rospy.Publisher("/arduino/valve2", Bool, queue_size=1) # Jar 2
        self.valve3_pub = rospy.Publisher("/arduino/valve3", Bool, queue_size=1) # Jar 3
        self.valve4_pub = rospy.Publisher("/arduino/valve4", Bool, queue_size=1) # Jar 4
        self.valve5_pub = rospy.Publisher("/arduino/valve5", Bool, queue_size=1) # Jar 5
        self.valve6_pub = rospy.Publisher("/arduino/valve6", Bool, queue_size=1) # Jar 6
        self.valve7_pub = rospy.Publisher("/arduino/valve7", Bool, queue_size=1) # Purge
        self.pump_pub = rospy.Publisher("/arduino/pump", Bool, queue_size=1)

        # Jar Volumes
        self.jar1_pub = rospy.Publisher("/volume/jar1", Float32, queue_size=1)
        self.jar2_pub = rospy.Publisher("/volume/jar2", Float32, queue_size=1)
        self.jar3_pub = rospy.Publisher("/volume/jar3", Float32, queue_size=1)
        self.jar4_pub = rospy.Publisher("/volume/jar4", Float32, queue_size=1)
        self.jar5_pub = rospy.Publisher("/volume/jar5", Float32, queue_size=1)
        self.jar6_pub = rospy.Publisher("/volume/jar6", Float32, queue_size=1)

        """Initialise Subscribers"""
        # Components
        self.valve1_sub = rospy.Subscriber("/arduino/valve1", Bool, self.handle_valve, (1), queue_size=1) # Jar 1
        self.valve2_sub = rospy.Subscriber("/arduino/valve2", Bool, self.handle_valve, (2), queue_size=1) # Jar 2
        self.valve3_sub = rospy.Subscriber("/arduino/valve3", Bool, self.handle_valve, (3), queue_size=1) # Jar 3
        self.valve4_sub = rospy.Subscriber("/arduino/valve4", Bool, self.handle_valve, (4), queue_size=1) # Jar 4
        self.valve5_sub = rospy.Subscriber("/arduino/valve5", Bool, self.handle_valve, (5), queue_size=1) # Jar 5
        self.valve6_sub = rospy.Subscriber("/arduino/valve6", Bool, self.handle_valve, (6), queue_size=1) # Jar 6
        self.valve7_sub = rospy.Subscriber("/arduino/valve7", Bool, self.handle_valve, (7), queue_size=1) # Purge
        self.pump_sub = rospy.Subscriber("/arduino/pump", Bool, self.handle_pump, queue_size=1)

        # Sensors
        self.flow_sub = rospy.Subscriber("/arduino/flow", Float32, self.handle_flow, queue_size=1) # Flow Rate Sensor
        self.temp_sub = rospy.Subscriber("/arduino/temp", Float32, self.handle_temp, queue_size=1) # Temperature
        self.depth_sub = rospy.Subscriber("/arduino/depth", Float32, self.handle_depth, queue_size=1) # Water Depth

        """Initialise Action Servers"""
        # GUI Auto Actions
        self.action_servers = {}
        self.action_servers['sample'] = SimpleActionServer('sample', SampleAction, self.exec_sample, auto_start=False)
        self.action_servers['purge'] = SimpleActionServer('purge', PurgeAction, self.exec_purge, auto_start=False)
        self.action_servers['stop'] = SimpleActionServer('stop', StopAction, self.exec_stop, auto_start=False)

        # GUI Manual Actions
        self.action_servers['pump'] = SimpleActionServer('pump', SetPumpAction, self.exec_set_pump, auto_start=False)
        for i in range(NUM_VALVES):
            name = 'valve' + str(i+1)
            self.action_servers[name] = SimpleActionServer(name, SetValveAction, self.exec_set_valve, auto_start=False)

        # Start Server
        for key, value in self.action_servers.items():
            print("Starting AS: " + key)
            value.start()

    """CLASS SERVER-SIDE FUNCTIONS"""
    def exec_sample(self, goal):
        print("Execute Sample")
        print(goal.jars) # List of Boolean of Size 6
        """
        # TODO
        1. Check if system is idle
        2. Open Specified Jar Valves. Ensure rest of valves are closed.
        3. Run the Pump
        4. Estimate Jar Capacity (dt * flow rate = volume). You might need to consider stuff like
            - Location of Jar in Water Block (ie, some jars are filled before others?)
            - Accuracy of flow rate can be compared with the pump specs && the flow sensor
        5. When a jar is 'filled', close the valve
        6. When all jars are filled, stop the process
        """


    def exec_purge(self, goal):
        # TODO HANDLE STOP REQUEST
        print("Execute Purge")
        """
        1. Check if System is idle
        2. Close all jar valves
        3. Open purge valve
        4. Turn on Pump
        """
        if self.sampler.state['pump']:
            # Pump is already running: not idle
            result = PurgeResult(
                success=False,
                message='System is not idle. Please wait for the pump to stop.'
            )
            self.action_servers['purge'].set_aborted(result)
        else:
            # Close Jar Valves
            self.set_jar_valves(False)

            # Open Purge Valve
            self.valve7_pub.publish(True)
            self.rate.sleep()

            # Turn on Pump
            self.pump_pub.publish(True)
            self.rate.sleep()

            # Wait 30 Seconds
            feedback = PurgeFeedback()
            counter = 0
            while counter < PURGE_DURATION * SLEEP_RATE:
                counter += 1
                feedback.eta = PURGE_DURATION - (counter // SLEEP_RATE)
                self.action_servers['purge'].publish_feedback(feedback)
                self.rate.sleep()

            # Turn off pump
            self.pump_pub.publish(False)
            self.rate.sleep()

            # Turn off Purge Valve
            self.valve7_pub.publish(False)
            self.rate.sleep()

            # Update Sampler State
            self.sampler.setPurged(True)

            # Return Message
            result = PurgeResult(
                success=True,
                message='Purging process completed successfully'
            )
            self.action_servers['purge'].set_succeeded(result)




    def exec_stop(self, goal):
        print("Received Message to Execute 'Stop'")
        """
        1. Stop Pump
        2. Close all valves
        """
        # Stop Pump
        self.pump_pub.publish(False)
        self.rate.sleep()

        # Close Jar Valves
        self.set_jar_valves(False)

        # Close Purge Valve
        self.valve7_pub.publish(False)
        self.rate.sleep()

        # Return Message
        result = StopResult(
            success=True,
            message='Pump & Valves stopped successfully'
        )
        self.action_servers['stop'].set_succeeded(result)


    def exec_set_pump(self, goal):
        print("Received Message to Set Pump to " + str(goal.state))

        # Set Pump State
        self.pump_pub.publish(goal.state)
        self.rate.sleep()

        # Return Message
        result = SetPumpResult(
            success=True,
            message='Pump state changed to ' + str(goal.state) + ' successfully'
        )
        self.action_servers['pump'].set_succeeded(result)


    def exec_set_valve(self, goal):
        print("Received Message to Set Valve " + str(goal.id) + " to " + str(goal.state))

        # Change Valve State
        if goal.id == 1:
            self.valve1_pub.publish(goal.state)
        elif goal.id == 2:
            self.valve2_pub.publish(goal.state)
        elif goal.id == 3:
            self.valve3_pub.publish(goal.state)
        elif goal.id == 4:
            self.valve4_pub.publish(goal.state)
        elif goal.id == 5:
            self.valve5_pub.publish(goal.state)
        elif goal.id == 6:
            self.valve6_pub.publish(goal.state)
        elif goal.id == 7:
            self.valve7_pub.publish(goal.state)

        # Return Message
        result = SetValveResult(
            success=True,
            message='Valve ' + str(goal.id) + ' state changed to ' + str(goal.state) + ' successfully'
        )
        self.action_servers['valve' + str(goal.id)].set_succeeded(result)


    """CLASS SUBSCRIBER CALLBACKS"""
    def handle_pump(self, msg):
        self.sampler.setState('pump', msg.data)

    def handle_valve(self, msg, id):
        self.sampler.setState('valve' + str(id), msg.data)

    def handle_flow(self, msg):
        self.sampler.processFlow(msg.data)

    def handle_temp(self, msg):
        self.sampler.processTemp(msg.data)

    def handle_depth(self, msg):
        self.sampler.processDepth(msg.data)

    """CLASS UTILITY FUNCTIONS"""
    def set_jar_valves(self, value):
        # Sleep (3hz) maybe so electronics doesn't do everything at once?
        self.valve1_pub.publish(value)
        self.rate.sleep()

        self.valve2_pub.publish(value)
        self.rate.sleep()

        self.valve3_pub.publish(value)
        self.rate.sleep()

        self.valve4_pub.publish(value)
        self.rate.sleep()

        self.valve5_pub.publish(value)
        self.rate.sleep()

        self.valve6_pub.publish(value)
        self.rate.sleep()




if __name__ == '__main__':
    rospy.init_node('main')
    Main()
    rospy.spin()