#!/usr/bin/env python3
""" Github에 나와있는 Publisher,Subscriber기준으로 library 목록 작성하였습니다.
    darknet_ros_msgs는 단독 테스트를 위해 주석처리해놨습니다.
    필요하다면 수정해주세요"""
import numpy as np
import rospy
import math
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Int8
# import cv2
from cv_bridge import CvBridge, CvBridgeError
# from darknet_ros_msgs.msg import BoundingBoxes


""" 여기서부터 MAVROS <-> DARKNET_ROS 토픽 코드들을 적어주세요"""
class Darknetinit():
    def __init__(self):
        super(Darknetinit, self).__init__()
        self.image = Image()
        self.bridge = CvBridge()
        self.point_cloud_sub = rospy.Subscriber('camera/depth/image_raw', Image, self.point_cloud_callback)

    def point_cloud_callback(self, data):
        self.image = data
        depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        depth_array = np.array(depth_image, dtype=np.float32)
        self.point_cloud = depth_array


