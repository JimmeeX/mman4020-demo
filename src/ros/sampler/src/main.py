#!/usr/bin/env python

import roslib
import rospy
from actionlib import SimpleActionServer

from std_msgs.msg import Bool
from sampler.msg import (
    PurgeAction,
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

class Main():
    def __init__(self):
        rospy.loginfo("Starting up Main")

        self.num_valves = 7
        self.rate = rospy.Rate(3)

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

        """Initialise Action Servers"""
        # GUI Auto Actions
        self.action_servers = {}
        self.action_servers['sample'] = SimpleActionServer('sample', SampleAction, self.exec_sample, auto_start=False)
        self.action_servers['purge'] = SimpleActionServer('purge', PurgeAction, self.exec_purge, auto_start=False)
        self.action_servers['stop'] = SimpleActionServer('stop', StopAction, self.exec_stop, auto_start=False)

        # GUI Manual Actions
        self.action_servers['pump'] = SimpleActionServer('pump', SetPumpAction, self.exec_set_pump, auto_start=False)
        for i in range(self.num_valves):
            name = 'valve' + str(i+1)
            self.action_servers[name] = SimpleActionServer(name, SetValveAction, self.exec_set_valve, auto_start=False)

        # Start Server
        for key, value in self.action_servers.items():
            print("Starting AS: " + key)
            value.start()


    def exec_sample(self, goal):
        print("Execute Sample")
        print(goal.jars)
        pass
        # data = message.data
        # if message.data == True:
        #     # TODO:
        #     """
        #     1. START PUMPING PROCESS
        #     2. Read Flow Rate
        #     """
        #     print("PUMPING")
        #     pass
        # else:
        #     # TODO: STOP PUMPING
        #     print("STOPPING")
        #     pass

    def exec_purge(self, goal):
        # TODO
        print("Execute Purge")
        """
        1. Check if System is idle
        2. Close all jar valves
        3. Open purge valve
        4. Turn on Pump
        """
        pass

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

    """Note that this will be run separately for each valve"""
    def exec_set_valve(self, goal):
        """
        Forward Message to Arduino
        """
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


    """CLASS UTILITY FUNCTIONS"""
    def set_jar_valves(self, value):
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