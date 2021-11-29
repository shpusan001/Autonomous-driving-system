#!/usr/bin/env python
# coding=utf-8

import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image
from robot_drive_controller import RobotDriveController
import numpy


class LineTracer2:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        # 추적하는 라인에 무게중심 점을 찍는 토픽 카메라 2개로 판단.
        # self.image_pub = rospy.Publisher(image_topic + "/circle", Image, queue_size=1)
        self.l_image_sub = rospy.Subscriber('left_camera/rgb/image_raw', Image, self.image_callback_l)
        self.r_image_sub = rospy.Subscriber('right_camera/rgb/image_raw', Image, self.image_callback_r)
        # self.t = image_topic
        self.lcx, self.rcx = 0, 0
        self.lcy, self.rcy = 0, 0
        self.stop_count = 0
        # 정지선 인식을 위한 중앙 카메라 사용.
        self.stop_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback_c)
        # stop line 을 판단하기 위한 area 크기.
        self.area = 0

    def image_callback_l(self, msg):
        origin_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # hsv 이미지로 변환.
        hsv_image = cv2.cvtColor(origin_image, cv2.COLOR_BGR2HSV)

        lower = numpy.array([10, 40, 135])
        upper = numpy.array([135, 135, 255])

        mask = cv2.inRange(hsv_image, lower, upper)
        h, w, d = origin_image.shape
        top = 3 * h / 8
        bot = top + 20
        mask[0:top, 0:w] = 0
        mask[bot:h, 0:w] = 0
        # 감지한 객체의 무게 중심을 화면에 찍는 변수
        # M은 리스트 형식이다.
        M = cv2.moments(mask)
        if M['m00'] > 0:
            self.lcx = int(M['m10'] / M['m00'])
            self.lcy = int(M['m01'] / M['m00'])
            cv2.circle(origin_image, (self.lcx, self.lcy), 20, (0, 255, 0), -1)
            # self.lcx = self.lcx - 320
        # cv2.imshow('window', origin_image)
        # cv2.waitKey(3)

    def image_callback_r(self, msg):
        origin_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # hsv 이미지로 변환.
        hsv_image = cv2.cvtColor(origin_image, cv2.COLOR_BGR2HSV)

        lower = numpy.array([0, 0, 200])  # 0, 0, 200
        upper = numpy.array([135, 135, 255])

        mask = cv2.inRange(hsv_image, lower, upper)
        h, w, d = origin_image.shape
        top = 3 * h / 8
        bot = top + 20
        mask[0:top, 0:w] = 0
        mask[bot:h, 0:w] = 0
        # 감지한 객체의 무게 중심을 화면에 찍는 변수
        # M은 리스트 형식이다.
        M = cv2.moments(mask)
        if M['m00'] > 0:
            self.rcx = int(M['m10'] / M['m00'])
            self.rcy = int(M['m01'] / M['m00'])
            cv2.circle(origin_image, (self.rcx, self.rcy), 15, (0, 255, 0), -1)
            # self.rcx = self.rcx - 320
        # cv2.imshow('window', origin_image)
        # cv2.waitKey(3)

    def image_callback_c(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # 흰색의 최대값과 최소값의 범위를 지정하여 마스크범위로 지정.
        lower_white = numpy.array([0, 0, 200])
        upper_white = numpy.array([0, 0, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # 마스크 범위 지정.
        h, w = mask.shape

        mask[0:h * 3 / 5, 0:w] = 0
        mask[h - (h / 8):h, 0:w] = 0
        mask[0:h, 0:w / 4] = 0
        mask[0:h, w - (w / 4):w] = 0

        # 127값을 넘는 값들은 255(흰색), 아래는 0으로 이진화한다.
        _, thr = cv2.threshold(mask, 127, 255, 0)

        # 이진화된 영상으로부터 객체를 찾는다.
        _, contours, _ = cv2.findContours(thr, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) <= 0:
            return  # not found

        cnt = contours[0]

        # area 에 contour 값을 넣어서 정지선을 처리한다.
        self.area = cv2.contourArea(cnt)

        # 이진화된 영상으로부터 사각형의 윤곽을 얻는다.
        x, y, w, h = cv2.boundingRect(cnt)
        mask = cv2.rectangle(mask, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.drawContours(mask, [cnt], 0, (255, 255, 0), 1)
        cv2.waitKey(3)
