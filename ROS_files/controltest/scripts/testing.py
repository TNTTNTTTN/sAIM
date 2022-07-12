#!/usr/bin/env python3
import ctypes
libgcc_s = ctypes.CDLL('libgcc_s.so.1')
import sys
import rospy
import math
import numpy as np
import os
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3, Twist
from mavros_msgs.msg import ParamValue, AttitudeTarget, PositionTarget, LandingTarget
from mavrosinit import Mavrosinit
from darknetinit import Darknetinit
from pymavlink import mavutil
from std_msgs.msg import Header, UInt16
from threading import Thread
# from tf.transformations import quaternion_from_euler

class Ctrtest(Darknetinit, Mavrosinit):
    def __init__(self, systype, wp1lat=0 , wp1lon=0, wp2lat=0, wp2lon=0, wp3lat=0, wp3lon=0):
        self.wp1lat = float(wp1lat)
        self.wp1lon = float(wp1lon)
        self.wp2lat = float(wp2lat)
        self.wp2lon = float(wp2lon)
        self.wp3lat = float(wp3lat)
        self.wp3lon = float(wp3lon)
        self.a = 6378137
        self.b = 6356752.3142
        self.f = (self.a - self.b) / self.a
        self.e_sq = self.f * (2 - self.f)
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
        self.att.type_mask = 7  # ignore body rate
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

    def geodetic_to_ecef(self, lat, lon, h):
        # (lat, lon) in WSG-84 degrees
        # h in meters
        lamb = math.radians(lat)
        phi = math.radians(lon)
        s = math.sin(lamb)
        N = self.a / math.sqrt(1 - self.e_sq * s * s)

        sin_lambda = math.sin(lamb)
        cos_lambda = math.cos(lamb)
        sin_phi = math.sin(phi)
        cos_phi = math.cos(phi)

        x = (h + N) * cos_lambda * cos_phi
        y = (h + N) * cos_lambda * sin_phi
        z = (h + (1 - self.e_sq) * N) * sin_lambda

        return x, y, z

    def ecef_to_enu(self, x, y, z, lat0, lon0, h0):
        lamb = math.radians(lat0)
        phi = math.radians(lon0)
        s = math.sin(lamb)
        N = self.a / math.sqrt(1 - self.e_sq * s * s)

        sin_lambda = math.sin(lamb)
        cos_lambda = math.cos(lamb)
        sin_phi = math.sin(phi)
        cos_phi = math.cos(phi)

        x0 = (h0 + N) * cos_lambda * cos_phi
        y0 = (h0 + N) * cos_lambda * sin_phi
        z0 = (h0 + (1 - self.e_sq) * N) * sin_lambda

        xd = x - x0
        yd = y - y0
        zd = z - z0

        xEast = -sin_phi * xd + cos_phi * yd
        yNorth = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd
        zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd

        return np.array([xEast, yNorth, zUp])

    def lla2enu(self, lat, lon, h, lat_ref, lon_ref, h_ref):
        x, y, z = self.geodetic_to_ecef(lat, lon, h)

        return self.ecef_to_enu(x, y, z, lat_ref, lon_ref, h_ref)

    def wptsetup(self,check):
        if check == 0:
            initlat = 0.0
            initlon = 0.0
            initheight = 0.0
            while not (initlat and initlon and initheight):
                initlat = float(self.global_position.latitude)
                initlon = float(self.global_position.longitude)
                initheight = float(self.global_position.altitude)
            local = np.array([self.local_position.pose.position.x, self.local_position.pose.position.y,
                     self.local_position.pose.position.z])
            wp1 = self.lla2enu(self.wp1lat, self.wp1lon, initheight+15, initlat, initlon, initheight) + local
            wp2 = self.lla2enu(self.wp2lat, self.wp2lon, initheight+15, initlat, initlon, initheight) + local
            wp3 = self.lla2enu(self.wp3lat, self.wp3lon, initheight+15, initlat, initlon, initheight) + local
            self.going = (local, (local[0] ,local[1] , 5), wp1, wp2, wp3)
        else:
            local = np.array([self.local_position.pose.position.x, self.local_position.pose.position.y,
                              self.local_position.pose.position.z])
            self.back = (local, self.going[4], self.going[3], self.going[2], self.going[1])

    def is_at_position(self, x, y, z, offset):
        """offset: meters"""
        rospy.logdebug(
            "current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
                self.local_position.pose.position.x, self.local_position.pose.
                position.y, self.local_position.pose.position.z))

        desired = np.array((x, y))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y))
        return (np.linalg.norm(desired - pos) < offset and (z - self.local_position.pose.position.z) < 1)

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
        # quaternion = quaternion_from_euler(yaw, 0, 0, axes="rzyx")
        # self.pos.pose.orientation = Quaternion(*quaternion)

        # does it reach the position in 'timeout' seconds?
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        for i in range(timeout * loop_freq):
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
        if posdiff > 50:
            if self.setupspeed < 10:
                self.setupspeed += 0.1
            else:
                self.setupspeed = 10
        else :
            if self.setupspeed > 0.2 * posdiff:
                self.setupspeed -= 0.1
            else :
                self.setupspeed = 0.2 * posdiff

        alpha = math.atan2(y - ori_y, x - ori_x)
        yaw = math.atan2(y - currenty, x - currentx)
        if np.sqrt((x - ori_x)**2 + (y - ori_y)**2) < 10:
            vector = self.uniformveccal(self.setupspeed, alpha) + self.sinkveccal(currentx, currenty, x, y, 5)
        else:
            vector = self.uniformveccal(self.setupspeed, alpha) + self.sinkveccal(currentx,currenty,x,y,600)
        if np.dot(directionvec, [currentx - x, currenty - y]) >= 0:
            vector = np.array([x - currentx, y - currenty])

        elif hasattr(self, 'Obj'):
            for i in range(len(self.Obj)):
                if i == 0:
                    continue
                elif np.dot(directionvec,[self.Obj[i][0] - x, self.Obj[i][1] - y]) >= 0:
                    continue
                elif abs(self.uniformpocal(currentx,currenty,alpha,self.setupspeed) - self.uniformpocal(self.Obj[i][0], self.Obj[i][1], alpha, self.setupspeed)) < self.setupspeed:
                    if currenty - self.Obj[i][1] == 0:
                        vector[1] += self.setupspeed
                    else:
                        vector[1] += 15*(currenty - self.Obj[i][1])/abs((currenty - self.Obj[i][1]))
                vector += self.doublitveccal(currentx, currenty, self.Obj[i][0], self.Obj[i][1], self.Obj[i][2], self.setupspeed, alpha)
                rospy.loginfo("{}, {}".format(i, self.doublitveccal(currentx, currenty, self.Obj[i][0], self.Obj[i][1], self.Obj[i][2], self.setupspeed, alpha)))
        vector = vector/np.linalg.norm(vector)
        self.vel.velocity.x = self.setupspeed * vector[0]
        self.vel.velocity.y = self.setupspeed * vector[1]
        self.vel.velocity.z = 0.8*(15 - currentz)
        if abs(yaw - self.vel.yaw) > math.pi:
            if yaw - self.vel.yaw > 0:
                self.vel.yaw -= (yaw - self.vel.yaw - (math.pi * 2))/10
            else:
                self.vel.yaw += (yaw - self.vel.yaw + (math.pi * 2))/10
        elif math.pi/4 < abs(yaw - self.vel.yaw):
            if yaw - self.vel.yaw > 0:
                self.vel.yaw += (yaw - self.vel.yaw)/10
        else:
            self.vel.yaw = yaw

        os.system('clear')
        print("velocity: {0}, {1}, {2}, {3}rad.".format(round(self.vel.velocity.x, 2), round(self.vel.velocity.y, 2), round(self.vel.velocity.z, 2), round(self.vel.yaw, 2)))
        print("curent position: {0}, {1}".format(round(self.local_position.pose.position.x, 2),round(self.local_position.pose.position.y, 2)))
        print("target_position: {0}, {1}".format(round(x, 2), round(y, 2)))

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

    def delivery(self):
        loop_freq = 10  # Hz
        rate = rospy.Rate(loop_freq)
        for _ in range(30 * loop_freq):
            self.vel.velocity.x = - 0.1 * self.local_position.pose.position.x / abs(self.local_position.pose.position.x)
            self.vel.velocity.y = - 0.1 * self.local_position.pose.position.y / abs(self.local_position.pose.position.y)
            self.vel.velocity.z = (5 - self.local_position.pose.position.z)
            self.vel.yaw += 0.1
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
        # positions = ((0, 0, 15), (-100, 55, 15), (-180, 99, 15), (-100, 50, 15),
        #              (0, 0, 15), (0, 0, 3))
        positions = ((0, 0, 1), (0, 0,10))
        for i in range(len(positions)):
            self.reach_position(positions[i][0], positions[i][1],
                                positions[i][2], 20)
        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   45, 0)
        self.set_arm(False, 5)

    def test_frequency(self):
        """Test offboard attitude control"""
        # make sure the simulation is ready to start the mission
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   10, -1)

        self.log_topic_vars()
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)
        timeout = 7
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        for i in range(timeout * loop_freq):
            self.att.thrust = 0.1 * i
            try:
                rate.sleep()
            except rospy.ROSException:
                quit()
        self.set_mode("AUTO.LAND", 5)
        self.set_arm(False, 5)

    def test_vel(self):
        self.mission_finish_pub = rospy.Publisher(
            'mission_status', UInt16, queue_size=1)

        # position = ((0,0,0), (0,0,15), (100,0,15), (100,100,15), (0, 0, 15))
        # self.Obj = ((),(25,3,20), (50,-5,15), (100,73,20), (50,50,20))
        pos = 1
        timeout = 180
        loop_freq = 10  # Hz
        rate = rospy.Rate(loop_freq)
        self.setupspeed = 0
        self.mission_finish_pub.publish(0)
        while not (self.state.mode == "OFFBOARD" and self.state.armed):
            self.mission_finish_pub.publish(0)
            print("\rwaiting", end="")
        self.wptsetup(0)
        for _ in range(timeout * loop_freq):
            if pos == len(self.going):
                break
            self.mission_finish_pub.publish(pos)
            self.potentialflow(self.going[pos-1][0], self.going[pos-1][1], self.going[pos-1][2], self.going[pos][0], self.going[pos][1], self.going[pos][2])
            if self.is_at_position(self.going[pos][0], self.going[pos][1], self.going[pos][2], self.radius):
                rospy.loginfo("position reached!")
                pos += 1

            try:
                rate.sleep()
            except rospy.ROSException:
                quit()
        self.mission_finish_pub.publish(6)
        self.delivery()

        pos = 1
        self.wptsetup(1)
        for _ in range(timeout * loop_freq):
            if pos == len(self.back):
                break
            self.mission_finish_pub.publish(5 - pos)
            self.potentialflow(self.back[pos-1][0], self.back[pos-1][1], self.back[pos-1][2], self.back[pos][0], self.back[pos][1], self.back[pos][2])
            if self.is_at_position(self.back[pos][0], self.back[pos][1], self.local_position.pose.position.z, self.radius):
                rospy.loginfo("position reached!")
                pos += 1

            try:
                rate.sleep()
            except rospy.ROSException:
                quit()
        self.mission_finish_pub.publish(5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   90, 0)
        self.mission_finish_pub.publish(5)

if __name__ == '__main__':
    rospy.init_node('controltest', anonymous=True)
    systype = int(input("Input system type: \n"))
    if systype == 1:
        offboard_control = Ctrtest(systype)
        offboard_control.test_posctl()
    elif systype == 2:
        offboard_control = Ctrtest(systype)
        offboard_control.test_frequency()
    elif systype == 3:
        lat1, lon1 = input("WPT1 GPS coordinate : \n").split()
        lat2, lon2 = input("WPT2 GPS coordinate : \n").split()
        lat3, lon3 = input("WPT3 GPS coordinate : \n").split()
        offboard_control = Ctrtest(systype, wp1lat=lat1, wp1lon=lon1, wp2lat=lat2, wp2lon=lon2, wp3lat=lat3, wp3lon=lon3)
        offboard_control.test_vel()
    else:
        print("wrong system type")
        sys.exit(1)

