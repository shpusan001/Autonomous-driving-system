#!/usr/bin/env python
# coding=utf-8

import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image
from robot_drive_controller import RobotDriveController
import numpy


class LineTracer:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.l_image_sub = rospy.Subscriber('left_camera/rgb/image_raw', Image, self.image_callback_l)
        self.r_image_sub = rospy.Subscriber('right_camera/rgb/image_raw', Image, self.image_callback_r)
        self.lcx, self.rcx = 0, 0
        self.lcy, self.rcy = 0, 0
        self.stop_count = 0
        self.stop_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback_c)
        self.area = 0

    def image_callback_l(self, msg):
        origin_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv_image = cv2.cvtColor(origin_image, cv2.COLOR_BGR2HSV)

        lower = numpy.array([10, 40, 135])
        upper = numpy.array([135, 135, 255])

        mask = cv2.inRange(hsv_image, lower, upper)
        h, w, d = origin_image.shape
        top = 3 * h / 8
        bot = top + 20
        mask[0:top, 0:w] = 0
        mask[bot:h, 0:w] = 0
        M = cv2.moments(mask)
        if M['m00'] > 0:
            self.lcx = int(M['m10'] / M['m00'])
            self.lcy = int(M['m01'] / M['m00'])
            cv2.circle(origin_image, (self.lcx, self.lcy), 20, (0, 255, 0), -1)

    def image_callback_r(self, msg):
        origin_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv_image = cv2.cvtColor(origin_image, cv2.COLOR_BGR2HSV)

        lower = numpy.array([10, 40, 135])  # 0, 0, 200
        upper = numpy.array([135, 135, 255])

        mask = cv2.inRange(hsv_image, lower, upper)
        h, w, d = origin_image.shape
        top = 3 * h / 8
        bot = top + 20
        mask[0:top, 0:w] = 0
        mask[bot:h, 0:w] = 0

        M = cv2.moments(mask)
        if M['m00'] > 0:
            self.rcx = int(M['m10'] / M['m00'])
            self.rcy = int(M['m01'] / M['m00'])
            cv2.circle(origin_image, (self.rcx, self.rcy), 15, (0, 255, 0), -1)

    def image_callback_c(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_white = numpy.array([0, 0, 200])
        upper_white = numpy.array([0, 0, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)

        h, w = mask.shape

        mask[0:h * 3 / 5, 0:w] = 0
        mask[h - (h / 8):h, 0:w] = 0
        mask[0:h, 0:w / 4] = 0
        mask[0:h, w - (w / 4):w] = 0

        _, thr = cv2.threshold(mask, 127, 255, 0)

        _, contours, _ = cv2.findContours(thr, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) <= 0:
            return  # not found

        cnt = contours[0]

        self.area = cv2.contourArea(cnt)

        x, y, w, h = cv2.boundingRect(cnt)
        mask = cv2.rectangle(mask, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.drawContours(mask, [cnt], 0, (255, 255, 0), 1)
        cv2.waitKey(3)


if __name__ == '__main__':
    rospy.init_node('lane_trace')
    line = LineTracer()
    drive_controller = RobotDriveController()
    rate = rospy.Rate(20)
    count = 0
    cnt = 0

    while not rospy.is_shutdown():
        print line.area
        # if count == 0:
        #     cx = (line.lcx -40 + line.rcx) / 2 - 320
        #     err = -float(cx) / 100
        #
        #     if line.area > 12000.0:
        #         drive_controller.set_velocity(0)
        #         drive_controller.set_angular(0)
        #         count = count + 1
        #         print('stop!')
        #         print(count)
        #         rospy.sleep(3)
        #
        #     if abs(err) >= 0.5:
        #         drive_controller.set_velocity(0.4)
        #         drive_controller.set_angular(err-0.4)
        #         drive_controller.drive()
        #
        #     elif abs(err) < 0.5:
        #         drive_controller.set_velocity(0.4)
        #         drive_controller.set_angular(err)
        #         drive_controller.drive()
        #
        # elif count == 1:
        #     cx = (line.lcx - 40 + line.rcx + 10) / 2 - 320
        #     err = -float(cx) / 100
        #
        #     if line.area > 12000.0:
        #         drive_controller.set_velocity(0)
        #         drive_controller.set_angular(0)
        #         count = count + 1
        #         print('stop!')
        #         print(count)
        #         rospy.sleep(3)
        #
        #     if abs(err) >= 0.5:
        #         drive_controller.set_velocity(0.4)
        #         drive_controller.set_angular(err)
        #         drive_controller.drive()
        #
        #     elif abs(err) < 0.5:
        #         drive_controller.set_velocity(0.4)
        #         drive_controller.set_angular(err)
        #         drive_controller.drive()
        #
        # elif count == 2:
        #     cx = (line.lcx - 40 + line.rcx + 10) / 2 - 330
        #     err = -float(cx) / 100
        #     if line.area > 9000.0:
        #         drive_controller.set_velocity(0)
        #         drive_controller.set_angular(0)
        #         count = count + 1
        #         print('stop!')
        #         print(count)
        #         if count == 4:
        #             drive_controller.set_velocity(0)
        #             drive_controller.set_angular(-0.2)
        #         rospy.sleep(3)
        #
        #     if abs(err) >= 0.5:
        #         drive_controller.set_velocity(0.6)
        #         drive_controller.set_angular(err)
        #         drive_controller.drive()
        #
        #     elif abs(err) < 0.5:
        #         drive_controller.set_velocity(0.6)
        #         drive_controller.set_angular(err)
        #         drive_controller.drive()
        #
        # rate.sleep()

    rospy.spin()

