#!/usr/bin/env python3
""" Github에 나와있는 Publisher,Subscriber기준으로 library 목록 작성하였습니다.
    darknet_ros_msgs는 단독 테스트를 위해 주석처리해놨습니다.
    필요하다면 수정해주세요"""
import rospy
import math
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from position_msgs.msg import ObjectPositions, ObjectPosition 
from geometry_msgs.msg import Quaternion
class Darknetinit():
    def __init__(self):
        super(Darknetinit, self).__init__()
        self.Object = ObjectPositions()
        self.Object_sub = rospy.Subscriber('/objects_position/message',ObjectPositions,self.Object_callback)
    def Object_callback(self,data):
        self.Object = ObjectPositions()
        classlen=len(data.object_positions)
        for i in range(classlen):
            self.Object.object_positions.append(self.cam_rotation(data.object_positions[i]))

    def cam_rotation(self, dataset):
        Object_refined = ObjectPosition()
        Object_refined.Class = dataset.Class
        Object_refined.x = dataset.z
        Object_refined.y = -dataset.x
        Object_refined.z = -dataset.y
        return Object_refined

    def body_rotation(self, dataset):
        # theta 15 degree rotation
        Cam_rotation = Quaternion(*[0, 0.1305262, 0, 0.9914449])

    """ 여기서부터 MAVROS <-> DARKNET_ROS 토픽 코드들을 적어주세요"""
