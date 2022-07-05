#!/usr/bin/env python3

PKG = 'px4'
import sys
import rospy
import math
import numpy as np
import pyproj
import scipy.spatial.transform
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3, Twist
from mavros_msgs.msg import ParamValue, AttitudeTarget, PositionTarget, LandingTarget
from mavrosinit import Mavrosinit
from darknetinit import Darknetinit
from pymavlink import mavutil
from six.moves import xrange
from std_msgs.msg import Header
from threading import Thread
from tf.transformations import quaternion_from_euler

class Ctrtest(Darknetinit, Mavrosinit):
    def __init__(self, systype, wp1lat=0 , wp1lon=0, wp2lat=0, wp2lon=0, wp3lat=0, wp3lon=0):
        self.wp1lat = float(wp1lat)
        self.wp1lon = float(wp1lon)
        self.wp2lat = float(wp2lat)
        self.wp2lon = float(wp2lon)
        self.wp3lat = float(wp3lat)
        self.wp3lon = float(wp3lon)
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

        elif systype == 3:
            self.vel = PositionTarget()
            self.vel.velocity = Vector3()
            self.radius = 0.5

            self.vel_setpoint_pub = rospy.Publisher(
                'mavros/setpoint_raw/local', PositionTarget, queue_size=1)
            self.vel_thread = Thread(target=self.send_vel, args=())
            self.vel_thread.daemon = True
            self.vel_thread.start()

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
        # self.att.orientation = Quaternion(*quaternion_from_euler(0, 0, 0, axes='rzyx'))
        self.att.orientation = self.local_position.pose.orientation
        self.att.thrust = 0.71
        # self.att.type_mask = 7  # ignore body rate
        self.att.type_mask = 7
        while not rospy.is_shutdown():
            self.att.header.stamp = rospy.Time.now()
            self.att_setpoint_pub.publish(self.att)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def send_vel(self):
        rate = rospy.Rate(10)  # Hz
        self.vel.header = Header()
        self.vel.header.frame_id = "base_footprint"
        self.vel.coordinate_frame = 1 # Local NED
        self.vel.type_mask = 7 # Ignore Body position

        while not rospy.is_shutdown():
            self.vel.header.stamp = rospy.Time.now()
            self.vel_setpoint_pub.publish(self.vel)

            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def lla2enu(self,lat, lon, alt, lat_org, lon_org, alt_org):
        transformer = pyproj.Transformer.from_crs(
            {"proj": 'latlong', "ellps": 'WGS84', "datum": 'WGS84'},
            {"proj": 'geocent', "ellps": 'WGS84', "datum": 'WGS84'},
        )
        x, y, z = transformer.transform(lon, lat, alt, radians=False)
        x_org, y_org, z_org = transformer.transform(lon_org, lat_org, alt_org, radians=False)
        vec = np.array([[x - x_org, y - y_org, z - z_org]]).T

        rot1 = scipy.spatial.transform.Rotation.from_euler('x', -(90 - lat_org),
                                                           degrees=True).as_matrix()  # angle*-1 : left handed *-1
        rot3 = scipy.spatial.transform.Rotation.from_euler('z', -(90 + lon_org),
                                                           degrees=True).as_matrix()  # angle*-1 : left handed *-1

        rotMatrix = rot1.dot(rot3)

        enu = rotMatrix.dot(vec).T.ravel()
        return enu.T

    def wptsetup(self):
        initlat = float(self.global_position.latitude)
        initlon = float(self.global_position.longitude)
        inithight = float(self.global_position.altitude)
        wp1 = self.lla2enu(self.wp1lat, self.wp1lon, inithight+15, initlat, initlon, inithight)
        wp2 = self.lla2enu(self.wp2lat, self.wp2lon, inithight+15, initlat, initlon, inithight)
        wp3 = self.lla2enu(self.wp3lat, self.wp3lon, inithight+15, initlat, initlon, inithight)
        self.going = ((wp1[0], wp1[1], wp1[2]), (wp2[0], wp2[1], wp2[2]), (wp3[0], wp3[1], wp3[2]))
        self.back = ((wp3[0], wp3[1], wp1[2]), (wp2[0], wp2[1], wp2[2]), (wp1[0], wp1[1], wp1[2]), (0, 0, 5))

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
        rospy.loginfo("global position checking | lat: {0}, lon: {1}, alt1:{2}, alt2:{3}"
                      .format(self.global_position.latitude,
                              self.global_position.longitude,
                              self.altitude.local, self.global_position.altitude))

        # For demo purposes we will lock yaw/heading to north.
        headingy = y - self.local_position.pose.position.y
        headingx = x - self.local_position.pose.position.x
        yaw = math.atan2(headingy,headingx)
        quaternion = quaternion_from_euler(yaw, 0, 0, axes="rzyx")
        self.pos.pose.orientation = Quaternion(*quaternion)

        # does it reach the position in 'timeout' seconds?
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        for i in xrange(timeout * loop_freq):
            if self.is_at_position(self.pos.pose.position.x,
                                   self.pos.pose.position.y,
                                   self.pos.pose.position.z, self.radius):
                rospy.loginfo("position reached | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                break

            try:
                rate.sleep()
            except rospy.ROSException:
                quit()

    def potentialflow(self,ori_x, ori_y, ori_z, x, y, z):
        """timeout(int): seconds"""
        currentx = self.local_position.pose.position.x
        currenty = self.local_position.pose.position.y
        currentz = self.local_position.pose.position.z
        posdiff = np.sqrt((x - currentx)**2 + (y-currenty)**2)
        directionvec = [x - ori_x, y - ori_y]

        if posdiff > 30:
            speed = 10
        else :
            speed = 0.25 * abs(posdiff)
        alpha = math.atan2(y - ori_y, x - ori_x)
        yaw = math.atan2(y - currenty, x - currentx)
        vector = self.uniformveccal(speed, alpha) + self.sinkveccal(currentx,currenty,x,y,300/posdiff)
        if np.dot(directionvec, [currentx - x, currenty - y]) >= 0:
            vector = np.array([x - currentx, y - currenty])
        elif hasattr(self, 'Obj'):
            for i in range(len(self.Obj)):
                if i == 0:
                    continue
                elif np.dot(directionvec,[self.Obj[i][0] - x, self.Obj[i][1] - y]) >= 0:
                    rospy.loginfo(i, np.dot(directionvec,[self.Obj[i][0] - x, self.Obj[i][1] - y]))
                    continue
                elif abs(self.uniformpocal(currentx,currenty,alpha,speed) - self.uniformpocal(self.Obj[i][0], self.Obj[i][1], alpha, speed)) < speed:
                    if currenty - self.Obj[i][1] == 0:
                        vector[1] += speed
                    else:
                        vector[1] += 15*(currenty - self.Obj[i][1])/abs((currenty - self.Obj[i][1]))
                vector += self.doublitveccal(currentx, currenty, self.Obj[i][0], self.Obj[i][1], self.Obj[i][2], speed, alpha)
                rospy.loginfo("{}, {}".format(i, self.doublitveccal(currentx, currenty, self.Obj[i][0], self.Obj[i][1], self.Obj[i][2], speed, alpha)))
        vector = vector/np.linalg.norm(vector)
        self.vel.velocity.x = speed * vector[0]
        self.vel.velocity.y = speed * vector[1]
        self.vel.velocity.z = 15 - currentz
        self.vel.yaw = yaw
        rospy.loginfo("velocity: {0}, {1}, {2}".format(round(self.vel.velocity.x), round(self.vel.velocity.y), round(self.vel.velocity.z)))
        rospy.loginfo("position: {0}, {1}".format(round(self.local_position.pose.position.x),round(self.local_position.pose.position.y)))

    def uniformpocal(self, x, y, alpha, speed):
        return speed*(y * math.cos(alpha) - x * math.sin(alpha))

    def uniformveccal(self, speed, alpha):
        return np.array([speed*math.cos(alpha), speed*math.sin(alpha)])

    def doublitpocal(self, x, y, objx, objy, range, speed):
        return -speed*(range ** 2)*(y-objy)/((x-objx) ** 2 + (y-objy) ** 2)

    def doublitveccal(self, x, y, objx, objy, range, speed, alpha):
        vecx = -speed*(range**2)*((x-objx)**2 - (y-objy)**2)/(((x-objx)**2 + (y-objy)**2)**2)
        vecy = -2*speed*(range**2)*((x-objx)*(y-objy))/(((x-objx)**2 + (y-objy)**2)**2)
        return np.array([vecx*math.cos(alpha) + vecy*math.sin(alpha), vecx*math.sin(alpha) + vecy*math.cos(alpha)])

    def sinkpocal(self, x, y, posx, posy, strength):
        return strength*math.atan2(y-posy,x-posx)

    def sinkveccal(self,x, y, posx, posy, strength):
        vecx = -strength * (x-posx)/((x-posx)**2 + (y-posy)**2)
        vecy = -strength * (y-posy)/((x-posx)**2 + (y-posy)**2)
        return np.array([vecx, vecy])

    def precisionlanding(self):
        self.landing_target_sub = rospy.Subscriber('mavros/landing_target/raw', LandingTarget,
                                            self.landingtarget_callback)
        self.landing_target = LandingTarget()
        self.set_mode('AUTO.PRECLAND',5)

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
        # positions = ((0, 0, 15), (-100, 55, 15), (-180, 99, 15), (-100, 50, 15),
        #              (0, 0, 15), (0, 0, 3))
        positions = ((0, 0, 1), (0, 0,10))

        for i in xrange(len(positions)):
            self.reach_position(positions[i][0], positions[i][1],
                                positions[i][2], 20)

        # self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   45, 0)
        self.set_arm(False, 5)

    def test_attctl(self):
        """Test offboard attitude control"""
        # make sure the simulation is ready to start the mission
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   10, -1)

        self.log_topic_vars()
        self.wptsetup()
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)
        rospy.loginfo("run mission")
        for i in range(len(self.going)):
            self.reach_position(self.going[i][0], self.going[i][1],
                                self.going[i][2], 30)

        for j in range(len(self.back)):
            self.reach_position(self.back[j][0], self.back[j][1],
                                self.back[j][2], 30)

        # does it cross expected boundaries in 'timeout' seconds?
        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   90, 0)
        self.set_arm(False, 5)

    def test_vel(self):
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   10, -1)

        self.log_topic_vars()
        self.wptsetup()
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)
        position = ((0,0,0), (0,0,15), (100,0,15), (100,100,15), (0, 0, 15))
        self.Obj = ((),(25,0,15),(50,-5,10),(95,73,20))
        pos = 1
        timeout = 180
        loop_freq = 10  # Hz
        rate = rospy.Rate(loop_freq)
        for i in xrange(timeout * loop_freq):
            if pos == len(position):
                break
            self.potentialflow(position[pos-1][0], position[pos-1][1], position[pos-1][2], position[pos][0], position[pos][1], position[pos][2])
            if self.is_at_position(position[pos][0], position[pos][1], self.local_position.pose.position.z,self.radius):
                rospy.loginfo("position reached!")
                pos += 1

            try:
                rate.sleep()
            except rospy.ROSException:
                quit()
        self.precisionlanding()
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   90, 0)
        self.set_arm(False, 5)

if __name__ == '__main__':
    rospy.init_node('controltest', anonymous=True)
    systype = int(input("Input system type: \n"))
    if systype == 1:
        offboard_control = Ctrtest(systype)
        offboard_control.test_posctl()
    elif systype == 2 or systype == 3:
        lat1, lon1 = input("WPT1 GPS coordinate : \n").split()
        lat2, lon2 = input("WPT2 GPS coordinate : \n").split()
        lat3, lon3 = input("WPT3 GPS coordinate : \n").split()
        offboard_control = Ctrtest(systype, wp1lat=lat1, wp1lon=lon1, wp2lat=lat2, wp2lon=lon2)
        if systype == 2:
            offboard_control.test_attctl()
        else:
            offboard_control.test_vel()
    else:
        print("wrong system type")
        sys.exit(1)

