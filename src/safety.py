#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class Safety:
    DRIVE_IN_TOPIC = '/drive'
    DRIVE_OUT_TOPIC = '/vesc/high_level/ackermann_cmd_mux/input/default'
    SCAN_TOPIC = '/scan'

    def __init__(self):
        self.sub_scan = rospy.Subscriber(self.SCAN_TOPIC,
                                         LaserScan,
                                         callback=self.scan_callback)
        self.sub_drive = rospy.Subscriber(self.DRIVE_IN_TOPIC,
                                          AckermannDriveStamped,
                                          callback=self.drive_callback)
        self.pub_drive = rospy.Publisher(self.DRIVE_OUT_TOPIC,
                                         AckermannDriveStamped,
                                         queue_size=1)

    def scan_callback(self, msg):
        self.scan = msg.ranges

    def drive_callback(self, msg):
        if self.is_safe(msg):
            self.pub_drive.publish(msg)

    def is_safe(self, msg):
        # TODO implement safety controller logic here
        return True

rospy.init_node('safety')
safety = Safety()
rospy.spin()
