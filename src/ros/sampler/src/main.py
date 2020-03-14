#!/usr/bin/env python

import rospy

from std_msgs.msg import Bool

class Main():
    def __init__(self):
        rospy.loginfo("Starting up Main")

        # Initialise Publishers
        self.open1_pub = rospy.Publisher("/arduino/open/1", Bool, queue_size=1)
        self.open2_pub = rospy.Publisher("/arduino/open/2", Bool, queue_size=1)
        self.open3_pub = rospy.Publisher("/arduino/open/3", Bool, queue_size=1)
        self.open4_pub = rospy.Publisher("/arduino/open/4", Bool, queue_size=1)
        self.open5_pub = rospy.Publisher("/arduino/open/5", Bool, queue_size=1)
        self.open6_pub = rospy.Publisher("/arduino/open/6", Bool, queue_size=1)
        self.purge_pub = rospy.Publisher("/arduino/purge", Bool, queue_size=1)
        self.pump_pub = rospy.Publisher("/arduino/pump", Bool, queue_size=1)

        # Initialise Subscribers
        self.start_sub = rospy.Subscriber("/commands/start", Bool, self.start, queue_size=1)

    def start(self, message):
        data = message.data
        if message.data == True:
            # TODO: START PUMPING
            print("PUMPING")
            pass
        else:
            # TODO: STOP PUMPING
            print("STOPPING")
            pass


if __name__ == '__main__':
    rospy.init_node('main')
    Main()
    rospy.spin()