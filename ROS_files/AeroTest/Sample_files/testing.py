#!/usr/bin/env python3

PKG = 'px4'
import rospy
import glob
import json
import math
import os
import sys
from mavros import mavlink
from mavros_msgs.msg import Mavlink, Waypoint, WaypointReached
from mavros_test_common import MavrosTestCommon
from pymavlink import mavutil
from six.moves import xrange
from threading import Thread
os.environ["PROJ_LIB"] = os.path.join(os.environ["CONDA_PREFIX"], "share", "proj")


class MavrosMissionTest(MavrosTestCommon):
    """
    Run a mission
    """

    def setUp(self):
        super(self.__class__, self).setUp()

        self.mission_item_reached = -1  # first mission item is 0
        self.mission_name = ""

        self.mavlink_pub = rospy.Publisher('mavlink/to', Mavlink, queue_size=1)
        self.mission_item_reached_sub = rospy.Subscriber(
            'mavros/mission/reached', WaypointReached,
            self.mission_item_reached_callback)

        # need to simulate heartbeat to prevent datalink loss detection
        self.hb_mav_msg = mavutil.mavlink.MAVLink_heartbeat_message(
            mavutil.mavlink.MAV_TYPE_GCS, 0, 0, 0, 0, 0)
        self.hb_mav_msg.pack(mavutil.mavlink.MAVLink('', 2, 1))
        self.hb_ros_msg = mavlink.convert_to_rosmsg(self.hb_mav_msg)
        self.hb_thread = Thread(target=self.send_heartbeat, args=())
        self.hb_thread.daemon = True
        self.hb_thread.start()


    def mission_item_reached_callback(self, data):
        if self.mission_item_reached != data.wp_seq:
            rospy.loginfo("mission item reached: {0}".format(data.wp_seq))
            self.mission_item_reached = data.wp_seq

    def send_heartbeat(self):
        rate = rospy.Rate(2)  # Hz
        while not rospy.is_shutdown():
            self.mavlink_pub.publish(self.hb_ros_msg)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_node', anonymous=True)

    name = "mavros_mission_test"
    if len(sys.argv) > 1:
        name += "-%s" % sys.argv[1]
    rostest.rosrun(PKG, name, MavrosMissionTest)