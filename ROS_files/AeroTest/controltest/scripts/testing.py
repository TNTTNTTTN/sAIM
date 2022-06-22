#!/usr/bin/env python3

PKG = 'px4'
import sys
import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3
from mavros_msgs.msg import ParamValue, AttitudeTarget
from initialset import Initialset
from pymavlink import mavutil
from six.moves import xrange
from std_msgs.msg import Header
from threading import Thread
from tf.transformations import quaternion_from_euler

class Ctrtest(Initialset):
    def __init__(self, systype):
        super(Ctrtest, self).__init__()
        if systype == 1:
        # Posistion control setup
            self.pos = PoseStamped()
            self.radius = 1

            self.pos_setpoint_pub = rospy.Publisher(
                'mavros/setpoint_position/local', PoseStamped, queue_size=1)

            # send setpoints in seperate thread to better prevent failsafe
            self.pos_thread = Thread(target=self.send_pos, args=())
            self.pos_thread.daemon = True
            self.pos_thread.start()
        elif systype == 2:
            # Attitude control setup
            self.att = AttitudeTarget()

            self.att_setpoint_pub = rospy.Publisher(
                'mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)

            # send setpoints in seperate thread to better prevent failsafe
            self.att_thread = Thread(target=self.send_att, args=())
            self.att_thread.daemon = True
            self.att_thread.start()

    def send_pos(self):
        rate = rospy.Rate(10)  # Hz
        self.pos.header = Header()
        self.pos.header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
            self.pos.header.stamp = rospy.Time.now()
            self.pos_setpoint_pub.publish(self.pos)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def send_att(self):
        rate = rospy.Rate(10)  # Hz
        self.att.body_rate = Vector3()
        self.att.header = Header()
        self.att.header.frame_id = "base_footprint"
        self.att.orientation = Quaternion(*quaternion_from_euler(0, 0,
                                                                 0, axes='rzyx'))
        self.att.thrust = 0.7
        self.att.type_mask = 7  # ignore body rate

        while not rospy.is_shutdown():
            self.att.header.stamp = rospy.Time.now()
            self.att_setpoint_pub.publish(self.att)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def is_at_position(self, x, y, z, offset):
        """offset: meters"""
        rospy.logdebug(
            "current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
                self.local_position.pose.position.x, self.local_position.pose.
                position.y, self.local_position.pose.position.z))

        desired = np.array((x, y, z))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z))
        return np.linalg.norm(desired - pos) < offset

    def reach_position(self, x, y, z, timeout):
        """timeout(int): seconds"""
        # set a position setpoint
        self.pos.pose.position.x = x
        self.pos.pose.position.y = y
        self.pos.pose.position.z = z
        rospy.loginfo(
            "attempting to reach position | x: {0}, y: {1}, z: {2} | current position x: {3:.2f}, y: {4:.2f}, z: {5:.2f}".
            format(x, y, z, self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z))
        rospy.loginfo("global position checking | lat: {0}, lon: {1}, alt:{2}"
                      .format(self.global_position.latitude,
                              self.global_position.longitude,
                              self.altitude.local))

        # For demo purposes we will lock yaw/heading to north.
        yaw_degrees = 0  # North
        yaw = math.radians(yaw_degrees)
        quaternion = quaternion_from_euler(yaw, 0, 0, axes="rzyx")
        self.pos.pose.orientation = Quaternion(*quaternion)

        # does it reach the position in 'timeout' seconds?
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        reached = False
        for i in xrange(timeout * loop_freq):
            if self.is_at_position(self.pos.pose.position.x,
                                   self.pos.pose.position.y,
                                   self.pos.pose.position.z, self.radius):
                rospy.loginfo("position reached | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                reached = True
                break

            try:
                rate.sleep()
            except rospy.ROSException:
                quit()

    def att_testing(self,x,y,z, timeout):
        """timeout(int): seconds"""
        # set a position setpoint
        # compare current position and designated position
        headingx = x - self.local_position.pose.position.x
        headingy = y - self.local_position.pose.position.y
        headingz = z - self.local_position.pose.position.z
        if abs(headingx) > 1 or abs(headingy) > 1:
            psi = math.atan2(headingy, headingx)

        if headingz > 0:
            self.att.thrust = 0.8
        else:
            self.att.thrust = 0.65

        rospy.loginfo(
            "attempting to reach position | x: {0}, y: {1}, z: {2} | current position x: {3:.2f}, y: {4:.2f}, z: {5:.2f}".
            format(x, y, z, self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z))
        rospy.loginfo("global position checking | lat: {0}, lon: {1}, alt:{2}"
                      .format(self.global_position.latitude,
                              self.global_position.longitude,
                              self.altitude.local))

        # does it reach the position in 'timeout' seconds?
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        reached = False
        for i in xrange(timeout * loop_freq):
            if self.is_at_position(self.pos.pose.position.x,
                                   self.pos.pose.position.y,
                                   self.pos.pose.position.z, self.radius):
                rospy.loginfo("position reached | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                reached = True
                break
            try:
                rate.sleep()
            except rospy.ROSException:
                quit()

    def test_posctl(self):
        """Test offboard position control"""

        # make sure the simulation is ready to start the mission
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   10, -1)

        self.log_topic_vars()
        # exempting failsafe from lost RC to allow offboard
        rcl_except = ParamValue(1<<2, 0.0)
        self.set_param("COM_RCL_EXCEPT", rcl_except, 5)
        self.set_arm(True, 5)
        self.set_mode("OFFBOARD", 5)
        rospy.loginfo("run mission")
        positions = ((0, 0, 0), (50, 50, 20), (50, -50, 20), (-50, -50, 20),
                     (0, 0, 20))

        for i in xrange(len(positions)):
            self.reach_position(positions[i][0], positions[i][1],
                                positions[i][2], 10)

        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   45, 0)

        self.set_arm(False, 5)

    def test_attctl(self):
        """Test offboard attitude control"""
        # boundary to cross
        boundary_x = 200
        boundary_y = 100
        boundary_z = 20

        # make sure the simulation is ready to start the mission
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   10, -1)

        self.log_topic_vars()
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)
        rospy.loginfo("run mission")
        rospy.loginfo("attempting to cross boundary | x: {0}, y: {1}, z: {2}".
                      format(boundary_x, boundary_y, boundary_z))
        # positions = ((0, 0, 0),(0, 0, 15), (50, 50, 15), (50, -50, 15), (-50, -50, 15),
        #              (0, 0, 0.5))
        # for i in xrange(len(positions)):
        #     self.reach_position(positions[i][0], positions[i][1],
        #                         positions[i][2], 30)

        # does it cross expected boundaries in 'timeout' seconds?
        timeout = 30  # (int) seconds
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        crossed = False
        for i in xrange(timeout * loop_freq):
            rospy.loginfo("current position | x: {0}, y: {1}, z: {2}".
                          format(self.local_position.pose.position.x, self.local_position.pose.position.y, self.local_position.pose.position.z))
            if (self.local_position.pose.position.x > boundary_x or
                    self.local_position.pose.position.y > boundary_y or
                    self.local_position.pose.position.z > boundary_z):
                rospy.loginfo("boundary crossed | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                crossed = True
                break
            try:
                rate.sleep()
            except rospy.ROSException as e:
                quit()

        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   90, 0)
        self.set_arm(False, 5)

if __name__ == '__main__':
    rospy.init_node('test_node', anonymous=True)
    if len(sys.argv) == 2:
        systype = int(sys.argv[1])
        offboard_control = Ctrtest(systype)
        if systype == 1:
            offboard_control.test_posctl()
        elif systype == 2:
            offboard_control.test_attctl()
    else:
        print("please input type for control: 1 = pos 2 = att")


