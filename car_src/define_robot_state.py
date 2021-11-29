#!/usr/bin/env python
# coding=utf-8

import rospy
from smach import State
from detect_blocking_bar import BlockDetector
from lane_follower import LineTracer
from lane_follower2 import LineTracer2
from robot_drive_controller import RobotDriveController
import time
import math
from detect_stop_sign import ImageConverter
from obstacle_stop import DetectObstacle


class SettingLane(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        pass

        return 'success'


class DetectBlockingBar(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        block_finder = BlockDetector()
        countfloor = 0
        while True:
            print(len(block_finder.contours))

            if len(block_finder.contours) == 0 and countfloor > 3:
                block_finder.drive_controller.set_velocity(1)

                print('go')

                start_time = time.time() + 4

                while True:
                    block_finder.drive_controller.drive()
                    if time.time() - start_time > 0:
                        break

                block_finder.drive_controller.set_velocity(0)

                return 'success'

            elif len(block_finder.contours) != 0 and countfloor > 3:
                block_finder.drive_controller.set_velocity(0)
                print('stop')

            else:
                print('detecting...')

            countfloor = countfloor + 1
            rospy.sleep(0.3)


class LaneTrace(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        line = LineTracer()
        drive_controller = RobotDriveController()
        line.lcx, line.rcx = 300, 300
        rospy.Rate(10)
        count = 0

        while not rospy.is_shutdown():
            if count <= 1:
                cx = (line.lcx + line.rcx) / 2 - 340
                err = -float(cx) / 100

                if line.area > 9000.0:
                    drive_controller.set_velocity(0)
                    drive_controller.set_angular(0)
                    count = count + 1
                    print('stop!')
                    print(count)
                    if count == 2:
                        drive_controller.set_velocity(0)
                        drive_controller.set_angular(-0.3)
                    rospy.sleep(3)

                if abs(err) >= 0.5:
                    drive_controller.set_velocity(0.6)
                    drive_controller.set_angular(err)
                    drive_controller.drive()

                elif abs(err) < 0.5:
                    drive_controller.set_velocity(0.6)
                    drive_controller.set_angular(err)
                    drive_controller.drive()

            elif count == 2:
                cx = (line.lcx - 40 + line.rcx) / 2 - 320
                err = -float(cx) / 100

                if line.area > 8700.0:
                    drive_controller.set_velocity(0)
                    drive_controller.set_angular(0)
                    count = count + 1
                    print('stop!')
                    print(count)
                    rospy.sleep(3)

                if line.rcx > 10:
                    if abs(err) >= 0.5:
                        drive_controller.set_velocity(0.4)
                        drive_controller.set_angular(err)
                        drive_controller.drive()

                    elif abs(err) < 0.5:
                        drive_controller.set_velocity(0.4)
                        drive_controller.set_angular(err)
                        drive_controller.drive()
                elif line.rcx <= 10:
                    drive_controller.set_velocity(0.1)
                    drive_controller.set_angular(-0.6)
                    drive_controller.drive()

            elif count == 3:
                cx = (line.lcx - 40 + line.rcx + 10) / 2 - 330
                err = -float(cx) / 100
                # print line.lcx
                if line.area > 9000.0:
                    drive_controller.set_velocity(0)
                    drive_controller.set_angular(0)
                    count = count + 1
                    print('stop!')
                    print(count)
                    rospy.sleep(3)

                if abs(err) >= 0.5:
                    drive_controller.set_velocity(0.6)
                    drive_controller.set_angular(err)
                    drive_controller.drive()

                elif abs(err) < 0.5:
                    drive_controller.set_velocity(0.6)
                    drive_controller.set_angular(err)
                    drive_controller.drive()
            else:
                # if count == 4:
                #     drive_controller.set_velocity(0)
                #     drive_controller.set_angular(-0.2)
                #     drive_controller.drive()
                print 'S course end!'
                return 'success'
        rospy.sleep()


class Straight(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        drive_controller = RobotDriveController()
        start_time = time.time()+5

        while not rospy.is_shutdown():
            drive_controller.set_velocity(0)
            drive_controller.set_angular(-0.15)
            drive_controller.drive()
            if time.time() - (start_time-4.8) > 0:
                break

        while not rospy.is_shutdown():
            drive_controller.set_velocity(1)
            drive_controller.set_angular(0)
            drive_controller.drive()
            if time.time() - start_time > 0:
                break
        return 'success'


class SCourseOneStep(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        line = LineTracer()
        drive_controller = RobotDriveController()
        rospy.Rate(10)
        count = 0

        while not rospy.is_shutdown():
            # print line.area
            if count == 0:
                cx = (line.lcx - 30 + line.rcx) / 2 - 320
                err = -float(cx) / 100

                if line.area > 6700.0:
                    drive_controller.set_velocity(0)
                    drive_controller.set_angular(0)
                    count = count + 1
                    print('stop!')
                    print(count)
                    # if count == 1:
                    #     drive_controller.set_velocity(0)
                    #     drive_controller.set_angular(1.2)
                    rospy.sleep(3)

                if abs(err) >= 0.5:
                    drive_controller.set_velocity(0.6)
                    drive_controller.set_angular(err-0.3)
                    drive_controller.drive()

                elif abs(err) < 0.5:
                    drive_controller.set_velocity(0.6)
                    drive_controller.set_angular(err-0.3)
                    drive_controller.drive()

            elif count == 1:
                cx = (line.lcx - 40 + line.rcx + 10) / 2 - 320
                err = -float(cx) / 100

                if line.area > 13000.0:
                    drive_controller.set_velocity(0)
                    drive_controller.set_angular(0)
                    count = count + 1
                    print('stop!')
                    print(count)
                    rospy.sleep(3)

                if abs(err) >= 0.5:
                    drive_controller.set_velocity(0.4)
                    drive_controller.set_angular(err)
                    drive_controller.drive()

                elif abs(err) < 0.5:
                    drive_controller.set_velocity(0.4)
                    drive_controller.set_angular(err)
                    drive_controller.drive()

            elif count == 2:
                return 'success'
        return 'success'


class SCourseTwoStep(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        line = LineTracer()
        drive_controller = RobotDriveController()
        count = 2
        start_time = time.time() + 2
        while not rospy.is_shutdown():
            rospy.Rate(10)

            print('go')
            while True:
                drive_controller.set_velocity(0.5)
                drive_controller.set_angular(0)
                drive_controller.drive()

                if time.time() - start_time > 0:
                    break

            print ('turn')
            while True:
                drive_controller.set_velocity(0)
                drive_controller.set_angular(-0.8)
                drive_controller.drive()

                if time.time() - start_time > 0:
                    break

            while not rospy.is_shutdown():
                cx = (line.lcx + line.rcx) / 2 - 320
                err = -float(cx) / 100
                rospy.Rate(10)
                # print count
                if line.area > 9000:
                    drive_controller.set_velocity(0)
                    drive_controller.set_angular(0)
                    count += 1
                    print ('stop!')
                    print (count)
                    rospy.sleep(3)

                if abs(err) >= 0.5:
                    drive_controller.set_velocity(0.4)
                    drive_controller.set_angular(err)
                    drive_controller.drive()

                elif abs(err) < 0.5:
                    drive_controller.set_velocity(0.4)
                    drive_controller.set_angular(err)
                    drive_controller.drive()

                if count == 3:
                    return 'success'
                    break


class Straight2(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        drive_controller = RobotDriveController()
        start_time = time.time() + 8.5

        print 'go straight'

        while not rospy.is_shutdown():
            drive_controller.set_velocity(0.7)
            drive_controller.set_angular(-0.06)
            drive_controller.drive()
            if time.time() - start_time > 0:
                break
        return 'success'


class TCourse(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        line = LineTracer()
        drive_controller = RobotDriveController()
        rospy.Rate(10)
        count = 0

        while not rospy.is_shutdown():
            if count == 0:
                cx = (line.lcx - 40 + line.rcx + 10) / 2 - 320
                err = -float(cx) / 100

                if line.area > 9000.0:
                    drive_controller.set_velocity(0)
                    drive_controller.set_angular(0)
                    count = count + 1
                    print('stop!')
                    print(count)
                    rospy.sleep(3)

                if abs(err) >= 0.5:
                    drive_controller.set_velocity(0.6)
                    drive_controller.set_angular(err)
                    drive_controller.drive()

                elif abs(err) < 0.5:
                    drive_controller.set_velocity(0.6)
                    drive_controller.set_angular(err)
                    drive_controller.drive()

            elif count == 1:
                return "success"



class Obstacle_Detect(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        line = LineTracer2()
        drive_controller = RobotDriveController()
        detect_obstacle = DetectObstacle()
        rospy.Rate(10)
        start_time = time.time() + 3
        cnt = 0
        count = 0

        while not rospy.is_shutdown():
            cx = (line.lcx + line.rcx) / 2 - 320
            err = -float(cx) / 100
            if abs(err) >= 0.5:
                drive_controller.set_velocity(0.6)
                drive_controller.set_angular(err)
                drive_controller.drive()

            elif abs(err) < 0.5:
                drive_controller.set_velocity(0.6)
                drive_controller.set_angular(err)
                drive_controller.drive()

            if time.time() - start_time > 0:
                break

        while not rospy.is_shutdown():
            cx = (line.lcx + line.rcx) / 2 - 320
            err = -float(cx)/100

            if cnt == 0:
                if detect_obstacle.range_ahead > 2 or detect_obstacle.range_right > 2 or \
                        ((math.isnan(detect_obstacle.range_ahead)) and math.isnan(detect_obstacle.range_right)):
                    if line.area > 9000:
                        drive_controller.set_velocity(0)
                        drive_controller.set_angular(0)
                        count += 1
                        print ('stop!')
                        print (count)
                        rospy.sleep(3)
                        return 'success'

                    if line.lcx > 10:
                        if abs(err) >= 0.5:
                            drive_controller.set_velocity(0.9)
                            drive_controller.set_angular(err)
                            drive_controller.drive()

                        elif abs(err) < 0.5:
                            drive_controller.set_velocity(0.9)
                            drive_controller.set_angular(err)
                            drive_controller.drive()
                    elif line.lcx <= 10:
                        drive_controller.set_velocity(0.2)
                        drive_controller.set_angular(0.7)
                        drive_controller.drive()
                else:
                    cnt += 1
                    drive_controller.set_velocity(0)
                    drive_controller.set_angular(0)
                    drive_controller.drive()
                    rospy.sleep(1)
                    print('stop! obstacle detect!!')
            else:
                if line.area > 9000:
                    drive_controller.set_velocity(0)
                    drive_controller.set_angular(0)
                    count += 1
                    print ('stop!')
                    print (count)
                    rospy.sleep(3)
                    return 'success'

                if line.lcx > 10:
                    if abs(err) >= 0.5:
                        drive_controller.set_velocity(0.9)
                        drive_controller.set_angular(err)
                        drive_controller.drive()

                    elif abs(err) < 0.5:
                        drive_controller.set_velocity(0.9)
                        drive_controller.set_angular(err)
                        drive_controller.drive()
                elif line.lcx <= 10:
                    drive_controller.set_velocity(0.2)
                    drive_controller.set_angular(0.7)
                    drive_controller.drive()


class Finish(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        line = LineTracer()
        drive_controller = RobotDriveController()
        count = 0
        start_time = time.time() + 12.2
        rospy.Rate(20)

        while True:
            # print line.area
            drive_controller.set_velocity(0.6)
            drive_controller.set_angular(-0.29)
            drive_controller.drive()
            if time.time() - start_time > 0:
                print 'turn end'
                break

        while not rospy.is_shutdown():
            print str(line.area)
            cx = (line.lcx + line.rcx) / 2 - 320
            err = -float(cx) / 100

            if count == 0:
                if 1500.0 > line.area > 1400.0:
                    drive_controller.set_velocity(0)
                    drive_controller.set_angular(0)
                    count = count + 1
                    print('stop!')
                    print(count)
                    rospy.sleep(3)

                if abs(err) >= 0.5:
                    drive_controller.set_velocity(0.4)
                    drive_controller.set_angular(err)
                    drive_controller.drive()

                elif abs(err) < 0.5:
                    drive_controller.set_velocity(0.4)
                    drive_controller.set_angular(err)
                    drive_controller.drive()
            elif count == 1:
                drive_controller.set_velocity(1)
                drive_controller.set_angular(0)
                drive_controller.drive()
                if time.time() - (time.time() + 3) > 0:
                    return 'success'


class Left(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        drive_controller = RobotDriveController()
        start_time = time.time() + 6.3

        print 'go left'

        while not rospy.is_shutdown():
            drive_controller.set_velocity(0.82)
            drive_controller.set_angular(0.28)
            drive_controller.drive()
            if time.time() - start_time > 0:
                break
        return 'success'


class DetectStopSign(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        line = LineTracer()
        drive_controller = RobotDriveController()
        stop_sign = ImageConverter()

        while not rospy.is_shutdown():
            # print len(stop_sign.counters)
            if stop_sign.detect:
                drive_controller.set_velocity(0)
                drive_controller.set_angular(0)
                rospy.sleep(3)
                return 'success'

            cx = (line.lcx - 30 + line.rcx) / 2 - 320
            err = -float(cx) / 100

            if abs(err) >= 0.5:
                drive_controller.set_velocity(0.6)
                drive_controller.set_angular(err)
                drive_controller.drive()

            elif abs(err) < 0.5:
                drive_controller.set_velocity(0.6)
                drive_controller.set_angular(err)
                drive_controller.drive()


class RightAngleParking(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        pass
        return 'success'


class ParallelParking(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        pass
        return 'success'


