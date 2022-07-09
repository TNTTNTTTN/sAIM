#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, PositionTarget, LandingTarget, ParamValue
from mavros_msgs.srv import CommandBool, ParamGet, ParamSet, SetMode
from pymavlink import mavutil
from sensor_msgs.msg import NavSatFix, Imu, BatteryState
from six.moves import xrange
from std_msgs.msg import UInt16
import os
# 콜백함수

def altitude_callback(data):
    global sub_topics_ready, altitude
    altitude = data

    # amsl has been observed to be nan while other fields are valid
    if not sub_topics_ready['alt'] and not math.isnan(data.amsl):
        sub_topics_ready['alt'] = True

def extended_state_callback(data):
    global sub_topics_ready, extended_state
    # if extended_state.landed_state != data.landed_state:
    #     rospy.loginfo("landed state changed from {0} to {1}".format(
    #         mavutil.mavlink.enums['MAV_LANDED_STATE']
    #         [extended_state.landed_state].name, mavutil.mavlink.enums[
    #             'MAV_LANDED_STATE'][data.landed_state].name))

    extended_state = data

    if not sub_topics_ready['ext_state']:
        sub_topics_ready['ext_state'] = True

def global_position_callback(data):
    global sub_topics_ready, global_position
    global_position = data

    if not sub_topics_ready['global_pos']:
        sub_topics_ready['global_pos'] = True

def imu_data_callback( data):
    global sub_topics_ready, imu_data
    imu_data = data

    if not sub_topics_ready['imu']:
        sub_topics_ready['imu'] = True

def home_position_callback(data):
    global sub_topics_ready, home_position
    home_position = data

    if not sub_topics_ready['home_pos']:
        sub_topics_ready['home_pos'] = True

def local_position_callback(data):
    global sub_topics_ready, local_position
    local_position = data

    if not sub_topics_ready['local_pos']:
        sub_topics_ready['local_pos'] = True

def battery_callback(data):
    global sub_topics_ready, battery
    battery = data

    if not sub_topics_ready['battery']:
        sub_topics_ready['battery'] = True

def state_callback(data):
    global sub_topics_ready, state
    # if state.armed != data.armed:
    #     rospy.loginfo("armed state changed from {0} to {1}".format(
    #         state.armed, data.armed))
    #
    # if state.connected != data.connected:
    #     rospy.loginfo("connected changed from {0} to {1}".format(
    #         state.connected, data.connected))
    #
    # if state.mode != data.mode:
    #     rospy.loginfo("mode changed from {0} to {1}".format(
    #         state.mode, data.mode))
    #
    # if state.system_status != data.system_status:
    #     rospy.loginfo("system_status changed from {0} to {1}".format(
    #         mavutil.mavlink.enums['MAV_STATE'][
    #             state.system_status].name, mavutil.mavlink.enums[
    #             'MAV_STATE'][data.system_status].name))

    state = data
    # mavros publishes a disconnected state message on init
    if not sub_topics_ready['state'] and data.connected:
        sub_topics_ready['state'] = True

def landingtarget_callback(data):
    global sub_topics_ready, landing_target
    landing_target = data

def local_vel_callback(data):
    global sub_topics_ready, local_vel
    local_vel = data

def mission_status_callback(data):
    global mission_status
    mission_status = data.data

# 기능성 함수
def set_arm( arm, timeout):
    """arm: True to arm or False to disarm, timeout(int): seconds"""
    rospy.loginfo("setting FCU arm: {0}".format(arm))
    loop_freq = 1  # Hz
    rate = rospy.Rate(loop_freq)
    for i in xrange(timeout * loop_freq):
        if state.armed == arm:
            rospy.loginfo("set arm success | seconds: {0} of {1}".format(
                i / loop_freq, timeout))
            break
        else:
            try:
                res = set_arming_srv(arm)
                if not res.success:
                    rospy.logerr("failed to send arm command")
            except rospy.ServiceException as e:
                rospy.logerr(e)

        try:
            rate.sleep()
        except rospy.ROSException:
            quit()

def set_mode( mode, timeout):
    """mode: PX4 mode string, timeout(int): seconds"""
    rospy.loginfo("setting FCU mode: {0}".format(mode))
    old_mode = state.mode
    loop_freq = 1  # Hz
    rate = rospy.Rate(loop_freq)
    for i in xrange(timeout * loop_freq):
        if state.mode == mode:
            rospy.loginfo("set mode success | seconds: {0} of {1}".format(
                i / loop_freq, timeout))
            break
        else:
            try:
                res = set_mode_srv(0, mode)  # 0 is custom mode
                if not res.mode_sent:
                    rospy.logerr("failed to send mode command")
            except rospy.ServiceException as e:
                rospy.logerr(e)

        try:
            rate.sleep()
        except rospy.ROSException:
            quit()

def set_param( param_id, param_value, timeout):
    """param: PX4 param string, ParamValue, timeout(int): seconds"""
    if param_value.integer != 0:
        value = param_value.integer
    else:
        value = param_value.real
    rospy.loginfo("setting PX4 parameter: {0} with value {1}".
                  format(param_id, value))
    loop_freq = 1  # Hz
    rate = rospy.Rate(loop_freq)
    param_set = False
    for i in xrange(timeout * loop_freq):
        try:
            res = set_param_srv(param_id, param_value)
            if res.success:
                rospy.loginfo("param {0} set to {1} | seconds: {2} of {3}".
                              format(param_id, value, i / loop_freq, timeout))
            break
        except rospy.ServiceException as e:
            rospy.logerr(e)

        try:
            rate.sleep()
        except rospy.ROSException:
            quit()

def wait_for_topics( timeout):
    """wait for simulation to be ready, make sure we're getting topic info
    from all topics by checking dictionary of flag values set in callbacks,
    timeout(int): seconds"""
    rospy.loginfo("waiting for subscribed topics to be ready")
    loop_freq = 1  # Hz
    rate = rospy.Rate(loop_freq)
    for i in xrange(timeout * loop_freq):
        if all(value for value in sub_topics_ready.values()):
            rospy.loginfo("simulation topics ready | seconds: {0} of {1}".
                          format(i / loop_freq, timeout))
            break

        try:
            rate.sleep()
        except rospy.ROSException:
            quit()

def wait_for_landed_state(desired_landed_state, timeout, index):
    rospy.loginfo("waiting for landed state | state: {0}, index: {1}".
                  format(mavutil.mavlink.enums['MAV_LANDED_STATE'][
                             desired_landed_state].name, index))
    loop_freq = 10  # Hz
    rate = rospy.Rate(loop_freq)
    landed_state_confirmed = False
    for i in xrange(timeout * loop_freq):
        if extended_state.landed_state == desired_landed_state:
            landed_state_confirmed = True
            rospy.loginfo("landed state confirmed | seconds: {0} of {1}".
                          format(i / loop_freq, timeout))
            break

        try:
            rate.sleep()
        except rospy.ROSException as e:
            quit()

def wait_for_mav_type(timeout):
    """Wait for MAV_TYPE parameter, timeout(int): seconds"""
    rospy.loginfo("waiting for MAV_TYPE")
    loop_freq = 1  # Hz
    rate = rospy.Rate(loop_freq)
    for i in xrange(timeout * loop_freq):
        try:
            res = get_param_srv('MAV_TYPE')
            if res.success:
                mav_type = res.value.integer
                rospy.loginfo(
                    "MAV_TYPE received | type: {0} | seconds: {1} of {2}".
                    format(mavutil.mavlink.enums['MAV_TYPE'][mav_type]
                           .name, i / loop_freq, timeout))
                break
        except rospy.ServiceException as e:
            rospy.logerr(e)

        try:
            rate.sleep()
        except rospy.ROSException:
            quit()

def log_topic_vars():
    """log the state of topic variables"""
    rospy.loginfo("========================")
    rospy.loginfo("===== topic values =====")
    rospy.loginfo("========================")
    rospy.loginfo("altitude:\n{}".format(altitude))
    rospy.loginfo("========================")
    rospy.loginfo("extended_state:\n{}".format(extended_state))
    rospy.loginfo("========================")
    rospy.loginfo("global_position:\n{}".format(global_position))
    rospy.loginfo("========================")
    rospy.loginfo("home_position:\n{}".format(home_position))
    rospy.loginfo("========================")
    rospy.loginfo("local_position:\n{}".format(local_position))
    rospy.loginfo("========================")
    rospy.loginfo("state:\n{}".format(state))
    rospy.loginfo("========================")

altitude = Altitude()
extended_state = ExtendedState()
global_position = NavSatFix()
imu_data = Imu()
home_position = HomePosition()
local_position = PoseStamped()
state = State()
local_vel = TwistStamped()
mav_type = None
battery = BatteryState()
landing_target = LandingTarget()
mission_status = UInt16()
sub_topics_ready = {
                key: False
                for key in [
                    'alt', 'ext_state', 'global_pos', 'home_pos', 'local_pos',
                    'state', 'imu', 'battery'
                ]
            }
service_timeout = 30
rospy.loginfo("waiting for ROS services")
try:
    rospy.wait_for_service('mavros/param/get', service_timeout)
    rospy.wait_for_service('mavros/param/set', service_timeout)
    rospy.wait_for_service('mavros/cmd/arming', service_timeout)
    rospy.wait_for_service('mavros/set_mode', service_timeout)
    rospy.loginfo("ROS services are up")
except rospy.ROSException:
    rospy.logerr("ROS service failure")
    quit()
get_param_srv = rospy.ServiceProxy('mavros/param/get', ParamGet)
set_param_srv = rospy.ServiceProxy('mavros/param/set', ParamSet)
set_arming_srv = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)


def subscriber():
    rospy.init_node('SafetyBehavior', anonymous=True)
    rospy.Subscriber('mavros/altitude', Altitude, altitude_callback)
    rospy.Subscriber('mavros/extended_state', ExtendedState, extended_state_callback)
    rospy.Subscriber('mavros/global_position/global', NavSatFix, global_position_callback)
    rospy.Subscriber('mavros/imu/data', Imu, imu_data_callback)
    rospy.Subscriber('mavros/home_position/home', HomePosition, home_position_callback)
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, local_position_callback)
    rospy.Subscriber('mavros/state', State, state_callback)
    rospy.Subscriber('mavros/battery', BatteryState, battery_callback)
    rospy.Subscriber('mavros/global_position/raw/gps_vel', TwistStamped, local_vel_callback)
    rospy.Subscriber('mavros/landing_target/raw', LandingTarget, landingtarget_callback)
    rospy.Subscriber('mission_status', UInt16, mission_status_callback)

class Selector(object):
    def __init__(self):
        self.child = []
        self.status = None

    def setup(self):
        self.status = False
        for i in range(len(self.child)):
            self.child[i].setup()
            if self.child[i].status:
                self.status = True
                break



class Sequence(object):
    def __init__(self):
        self.child = []
        self.status = None
    def setup(self):
        self.status = True
        for i in range(len(self.child)):
            self.child[i].setup()
            if not self.child[i].status:
                self.status = False
                break

class Armcheck():
    def __init__(self):
        self.status = None
    def setup(self):
        if not state.armed:
            self.status = True
        else:
            self.status = False

class Syscheck():
    def __init__(self, checktype):
        """checktype : 0 (initial), 1(continuous), 2(only gps), 3(landing state)"""
        self.type = checktype
        self.status = None

    def setup(self):
        if self.type == 0:
            if not all(value for value in sub_topics_ready.values()):
                self.status = False

            elif not extended_state.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
                self.status = False

        if not state.connected:
            self.status = False

        elif state.system_status > 4:
            self.status = False

        elif battery.percentage < 0.6:
            self.status = False

        elif global_position.status.status < 0:
            if self.type == 2:
                self.status = True
            else:
                self.status = False
        else:
            self.status = True

class Armset():
    def __init__(self,typecheck):
        self.type = typecheck
    def setup(self):
        if self.type == 0:
            rcl_except = ParamValue(1 << 2, 0.0)
            set_param("COM_RCL_EXCEPT", rcl_except, 5)
            while True:
                res = set_arming_srv(True)
                if res.success:
                    self.status = True
                    break
        else:
            while True:
                res = set_arming_srv(False)
                if res.success:
                    self.status = True
                    break

class Mission():
    def __init__(self):
        self.status = None

    def setup(self):
        if (state.mode == 'OFFBOARD' and mission_status in (1,2,3)) \
            or (state.mode == "AUTO.PRECLAND" and mission_status == 4):
            self.status = True
        elif state.mode == 'OFFBOARD' and mission_status == 4:
            res = set_mode_srv(0, 'AUTO.PRECLAND')
            if res.mode_sent:
                self.status = True
        else:
            res = set_mode_srv(0, 'OFFBOARD')
            if res.mode_sent:
                self.status = True

class Modechange() :
    def __init__(self):
        self.status = None

    def setup(self):
        if state.mode == 'AUTO.LOITER':
            self.status = True
        else:
            res = set_mode_srv(0, 'AUTO.LOITER')
            if res.mode_sent:
                self.status = True

class Modecheck() :
    def __init__(self, mode):
        self.status = None
        self.mode = mode
    def setup(self):
        if type(self.mode) == type(list()):
            self.status = False
            for i in range(len(self.mode)):
                if state.mode == self.mode[i]:
                    self.status = True
                    break

        else:
            if state.mode == self.mode:
                self.status = True
            else:
                self.status = False

def preorder(node):
    if node:
        node.setup()
    os.system('clear')
    print("*************CURRENT STATUS*************")
    print("ARMED = {}".format(state.armed))
    print("MODE = {}".format(state.mode))
    print("BATTERY = {}%".format(round(battery.percentage*100, 2)))
    if state.system_status < 5:
        print("SYSTEM STATUS = FINE")
    else:
        print("SYSTEM STATUS = CRITICAL")
    if mission_status == 0:
        print("MISSION STATUS: SETUP")
    elif mission_status == 1:
        print("MISSION STATUS = WPT1")
    elif mission_status == 2:
        print("MISSION STATUS = WPT2")
    elif mission_status == 3:
        print("MISSION STATUS = WPT3")
    elif mission_status == 4:
        print("MISSION STATUS = LANDING")
    elif mission_status == 5:
        print("MISSION STATUS = FINISH!")
    else:
        print("NO MISSION INPUT!")

print("debug point2")

def rootset():
    initsel = Selector()
    manumode = Modecheck("POSCTL")
    mainsel = Selector()
    landseq = Sequence()
    armseq = Sequence()
    misseq = Sequence()
    gpsseq = Sequence()
    loitermode = Modechange()
    landcheck = Modecheck(["AUTO.LAND", "AUTO.PRECLAND"])
    landstacheck = Syscheck(3)
    disarm = Armset(1)
    armcheck = Armcheck()
    initcheck = Syscheck(0)
    arming = Armset(0)
    maincheck = Syscheck(1)
    main = Mission()
    gpscheck = Syscheck(2)
    nongps = Mission()
    initsel.child.extend([manumode, mainsel, loitermode])
    mainsel.child.extend([armseq, misseq, gpsseq])
    # landseq.child.extend([landcheck, landstacheck, disarm])
    armseq.child.extend([armcheck, initcheck, arming])
    misseq.child.extend([maincheck, main])
    gpsseq.child.extend([gpscheck, nongps])
    return initsel

if __name__ == '__main__':
    # 변수 초기화
    # ROS subscribers
    subscriber()
    rate = rospy.Rate(5)
    root = rootset()
    while not rospy.is_shutdown():
        if mission_status == 5:
            res = set_arming_srv(False)
            quit()
        preorder(root)
        rate.sleep()


