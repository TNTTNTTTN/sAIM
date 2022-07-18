#!/usr/bin/env python3

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, PositionTarget, LandingTarget, ParamValue
from mavros_msgs.srv import CommandBool, ParamGet, ParamSet, SetMode
from pymavlink import mavutil
from sensor_msgs.msg import NavSatFix, Imu, BatteryState, Image
from six.moves import xrange
from std_msgs.msg import UInt16, Bool
import os

# 콜백함수
def altitude_callback(data):
    global sub_topics_ready, altitude
    altitude = data

    if not sub_topics_ready['alt'] and not math.isnan(data.amsl):
        sub_topics_ready['alt'] = True

def extended_state_callback(data):
    global sub_topics_ready, extended_state
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
#
# def distance_callback(data):
#     global depth_array, image
#     image = data
#     depth_image = bridge.imgmsg_to_cv2(data, "32FC1")
#     depth_array = np.array(depth_image, dtype=np.float32)
#
# def is_object_callback(data):
#     global objectcheck
#     objectcheck = data
#
# def safetyrangecheck(depth):
#     if depth[depth < 18].size == 0:
#         return False
#     else:
#         array = np.min(depth[depth < 18], axis=0)
#         if True in array[array > 0 and array < 1.5]:
#             return True
#         else:
#             return False

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
# bridge = CvBridge()
objectcheck = Bool()
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
    # rospy.Subscriber('camera/depth/image_raw', Image, distance_callback)
    rospy.Subscriber('mission_status', UInt16, mission_status_callback)
    # rospy.Subscriber('is_object',Bool, is_object_callback)

def batcal():
    return 1 - 0.05*(25.2 - battery.voltage)/0.3 
def mission_check():
    if mission_status == 0:
        print("MISSION STATUS: SETUP")
    elif mission_status == 1:
        print("MISSION STATUS: TAKEOFF")
    elif mission_status == 2:
        print("MISSION STATUS = WPT1")
    elif mission_status == 3:
        print("MISSION STATUS = WPT2")
    elif mission_status == 4:
        print("MISSION STATUS = WPT3")
    elif mission_status == 5:
        print("MISSION STATUS = LANDING")
    elif mission_status == 6:
        print("MISSION STATUS = DELIVERING")
    elif mission_status == 7:
        print("MISSION STATUS = FINISH!")
    else:
        print("NO MISSION INPUT!")
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
            print("FCU CONNECTON FAILED")
            self.status = False

        elif state.system_status > 4:
            print("SYSTEM STATUS CRITICAL")
            self.status = False

        a = batcal()

        elif a < 0.6:
            if self.type == 0:
                print("LOW BATTERY STATUS")
                self.status = False
            else :
                if a < 0.1:
                    print("CRITICAL LOW BATTERY STATUS")
                    self.status = False
                else : 
                    print("WARNING: LOW BATTERY STATUS")


        elif global_position.status.status < 0:
            print("GPS NOT FIXED")
            if self.type == 2:

                self.status = True
            else:
                self.status = False
        else:
            print("SYSTEM STATUS = FINE")
            self.status = True


class Armset():
    def __init__(self,typecheck):
        self.type = typecheck
    def setup(self):
        if self.type == 0:
            rcl_except = ParamValue(1 << 2, 0.0)
            set_param_srv("COM_RCL_EXCEPT", rcl_except)
            a = input("type \"ARM\" to set arm: \n")
            while a == "ARM":
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
    def __init__(self, check):
        self.status = None
        self.check = check
    def setup(self):
        if self.check == 1:
            if objectcheck:
                print("NON-GPS flight: OBJECT BASED MOVEMENT")
            else:
                print("MISSION PROCEED FAILURE : NO OBJECT FOR ESTIMATION")
                self.status = False
                return
        # if mission_status in (2, 3) and safetyrangecheck(depth_array):
        #     print("MISSION PROCEED FAILURE : OBJECT TOO CLOSE")
        #     self.status = False
        # else:
        if (state.mode == 'OFFBOARD' and mission_status in (1,2,3)) \
            or (state.mode == "AUTO.PRECLAND" and mission_status == 5):
            self.status = True
        elif state.mode == 'OFFBOARD' and mission_status == 5:
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
    os.system('clear')
    print("*************CURRENT STATUS*************")
    print("ARMED = {}".format(state.armed))
    print("MODE = {}".format(state.mode))
    print("BATTERY = {}%".format(round(batcal() * 100, 2)))
    if node:
        node.setup()
    mission_check()

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
    main = Mission(0)
    gpscheck = Syscheck(2)
    nongps = Mission(1)
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
        if mission_status == 7:
            res = set_arming_srv(False)
            quit()
        preorder(root)
        rate.sleep()


