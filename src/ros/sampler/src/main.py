#!/usr/bin/env python

import roslib
roslib.load_manifest('sampler')
import rospy
from actionlib import SimpleActionServer

from std_msgs.msg import Bool
from sampler.msg import (
    PurgeAction,
    SampleAction,
    SetPumpAction,
    SetValveAction,
    StopAction
)

class Main():
    def __init__(self):
        rospy.loginfo("Starting up Main")

        self.num_valves = 7

        """Initialise Publishers"""
        # Arduino Control Panel
        self.open1_pub = rospy.Publisher("/arduino/open/1", Bool, queue_size=1)
        self.open2_pub = rospy.Publisher("/arduino/open/2", Bool, queue_size=1)
        self.open3_pub = rospy.Publisher("/arduino/open/3", Bool, queue_size=1)
        self.open4_pub = rospy.Publisher("/arduino/open/4", Bool, queue_size=1)
        self.open5_pub = rospy.Publisher("/arduino/open/5", Bool, queue_size=1)
        self.open6_pub = rospy.Publisher("/arduino/open/6", Bool, queue_size=1)
        self.purge_pub = rospy.Publisher("/arduino/purge", Bool, queue_size=1)
        self.pump_pub = rospy.Publisher("/arduino/pump", Bool, queue_size=1)

        """Initialise Subscribers"""

        # # Auto Commands
        # self.sample_sub = rospy.Subscriber("/commands/sample", Bool, self.start_sampling, queue_size=1)
        # self.purge_sub = rospy.Subscriber("/commands/purge", Bool, self.purge_pump, queue_size=1)
        # self.stop_sub = rospy.Subscriber("/commands/stop", Bool, self.stop, queue_size=1)

        # # Manual Commands
        # self.pump_on = rospy.Subscriber("/commands/pump_on", Bool, self.pump_on, queue_size=1)
        # self.open1_sub = rospy.Subscriber("/commands/valve_on/1", Bool, self.valve_on, (1), queue_size=1)
        # self.open2_sub = rospy.Subscriber("/commands/valve_on/2", Bool, self.valve_on, (2), queue_size=1)
        # self.open3_sub = rospy.Subscriber("/commands/valve_on/3", Bool, self.valve_on, (3), queue_size=1)
        # self.open4_sub = rospy.Subscriber("/commands/valve_on/4", Bool, self.valve_on, (4), queue_size=1)
        # self.open5_sub = rospy.Subscriber("/commands/valve_on/5", Bool, self.valve_on, (5), queue_size=1)
        # self.open6_sub = rospy.Subscriber("/commands/valve_on/6", Bool, self.valve_on, (6), queue_size=1)
        # self.open7_sub = rospy.Subscriber("/commands/valve_on/7", Bool, self.valve_on, (7), queue_size=1)

        """Initialise Action Servers"""
        # GUI Auto Actions
        self.action_servers = {}
        self.action_servers['sample'] = SimpleActionServer('sample', SampleAction, self.exec_sample, auto_start=False)
        self.action_servers['purge'] = SimpleActionServer('purge', PurgeAction, self.exec_purge, auto_start=False)
        self.action_servers['stop'] = SimpleActionServer('stop', StopAction, self.exec_stop, auto_start=False)

        # GUI Manual Actions
        self.action_servers['pump'] = SimpleActionServer('pump', SetPumpAction, self.exec_set_pump, auto_start=False)
        for i in range(self.num_valves):
            name = 'valve' + str(i)
            self.action_servers[name] = SimpleActionServer(name, SetValveAction, self.exec_set_valve, auto_start=False)


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
        pass

    def exec_stop(self, goal):
        # TODO
        print("Execute Stop")
        pass

    def exec_set_pump(self, goal):
        print("Execute Set Pump")
        print(goal.state)
        # TODO
        pass

    """Note that this will be run separately for each valve"""
    def exec_set_valve(self, goal):
        """
        Forward Message to Arduino
        """
        print("Execute Set Valve")
        print(goal.id)
        print(goal.state)
        #
        pass


if __name__ == '__main__':
    rospy.init_node('main')
    Main()
    rospy.spin()