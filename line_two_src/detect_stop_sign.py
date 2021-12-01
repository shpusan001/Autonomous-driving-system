#! /usr/bin/env python
# coding=utf-8

import rospy
import cv2, cv_bridge
import numpy as np
from robot_drive_controller import RobotDriveController
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class ImageConverter:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.match_pub = rospy.Publisher("matches/is_block", Bool, queue_size=1)
        self.image_sub = rospy.Subscriber("camera/rgb/image_raw", Image, self.image_callback)
        self.robot_drive_controller = RobotDriveController()
        self.counters = []
        self.detect = False

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_red = np.array([0, 50, 50])
        upper_red = np.array([15, 255, 255])
        gray_img = cv2.inRange(hsv, lower_red, upper_red)

        h, w = gray_img.shape
        stop_sign_mask = gray_img
        stop_sign_mask[0:0, 0:w] = 0
        stop_sign_mask[10:h, 0:w] = 0
        stop_sign_mask, self.counters, hierarchy = cv2.findContours(stop_sign_mask, cv2.RETR_TREE,
                                                                    cv2.CHAIN_APPROX_SIMPLE)

        if len(self.counters) > 1:
            self.detect = True
