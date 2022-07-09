#!/usr/bin/env python3
""" Github에 나와있는 Publisher,Subscriber기준으로 library 목록 작성하였습니다.
    darknet_ros_msgs는 단독 테스트를 위해 주석처리해놨습니다.
    필요하다면 수정해주세요"""
import rospy
import math
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
# from darknet_ros_msgs.msg import BoundingBoxes


class Darknetinit():
    def __init__(self):
        super(Darknetinit, self).__init__()
    """ 여기서부터 MAVROS <-> DARKNET_ROS 토픽 코드들을 적어주세요"""