#!/usr/bin/env python3
""" Github에 나와있는 Publisher,Subscriber기준으로 library 목록 작성하였습니다.
    darknet_ros_msgs는 단독 테스트를 위해 주석처리해놨습니다.
    필요하다면 수정해주세요"""
import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from position_msgs.msg import ObjectPositions, ObjectPosition
from geometry_msgs.msg import Quaternion
class Darknetinit():
    def __init__(self):
        super(Darknetinit, self).__init__()
        self.Object = ObjectPositions()
        self.Object_sub = rospy.Subscriber('/objects_position/message', ObjectPositions,self.Object_callback)
    def Object_callback(self,data):
        self.Object = ObjectPositions()
        classlen=len(data.object_positions)
        for i in range(classlen):
            if data.object_positions[i].Class == "Person":
                self.Object.object_positions.append(self.cam_rotation(data.object_positions[i]))

    def cam_rotation(self, dataset):
        Object_refined = ObjectPosition()
        Object_refined.Class = dataset.Class
        Object_refined.x = dataset.z
        Object_refined.y = -dataset.x
        Object_refined.z = -dataset.y
        Object_refined.x, Object_refined.y, Object_refined.z = self.body_rotation(Object_refined)
        return Object_refined

    def body_rotation(self, dataset):
        # theta 15 degree rotation
        cam_rotation = Quaternion(*[0, 0.1305262, 0, 0.9914449])
        multiple = np.matmul(self.quat2dcm(cam_rotation), [[dataset.x], [dataset.y], [dataset.z]])
        return multiple[0], multiple[1], multiple[2]

    def quat2dcm(self,quat:Quaternion):
        q0 = quat.w
        q1 = quat.x
        q2 = quat.y
        q3 = quat.z
        one = [q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)]
        two = [2*(q1*q2+q0*q3), q0**2+q2**2-q1**2-q3**2, 2*(q2*q3-q0*q1)]
        three = [2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0**2+q3**2-q1**2-q2**2]
        return [one,two,three]
    """ 여기서부터 MAVROS <-> DARKNET_ROS 토픽 코드들을 적어주세요"""