#!/usr/bin/env python
# coding=utf-8

import rospy
from smach import State
from detect_blocking_bar import BlockDetector
from lane_follower import LineTracer
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

    saved_count = 8

    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        line = LineTracer()
        drive_controller = RobotDriveController()
        rospy.Rate(10)
        count = LaneTrace.saved_count

        while not rospy.is_shutdown():
            #first stopline
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
                    drive_controller.set_velocity(0.8)
                    drive_controller.set_angular(err)
                    drive_controller.drive()

                elif abs(err) < 0.5:
                    drive_controller.set_velocity(0.8)
                    drive_controller.set_angular(err)
                    drive_controller.drive()

            #second stop line (z course start)
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
                        drive_controller.set_velocity(0.44)
                        drive_controller.set_angular(err)
                        drive_controller.drive()

                    elif abs(err) < 0.5:
                        drive_controller.set_velocity(0.44)
                        drive_controller.set_angular(err)
                        drive_controller.drive()
                elif line.rcx <= 10:
                    drive_controller.set_velocity(0.1)
                    drive_controller.set_angular(-0.6)
                    drive_controller.drive()

            # third stop line (z course end)
            elif count == 3:
                cx = (line.lcx - 40 + line.rcx + 10) / 2 - 330
                err = -float(cx) / 100
                if line.area > 9100.0:
                    drive_controller.set_velocity(0)
                    drive_controller.set_angular(0)
                    count = count + 1
                    print('stop!')
                    print(count)
                    rospy.sleep(3)

                if abs(err) >= 0.5:
                    drive_controller.set_velocity(0.63)
                    drive_controller.set_angular(err)
                    drive_controller.drive()

                elif abs(err) < 0.5:
                    drive_controller.set_velocity(0.63)
                    drive_controller.set_angular(err)
                    drive_controller.drive()

            #fourth stop line (cross course start)
            elif count == 4:
                cx = (line.lcx - 40 + line.rcx + 10) / 2 - 330
                err = -float(cx) / 100
                if line.area > 9000.0:
                    drive_controller.set_velocity(0)
                    drive_controller.set_angular(0)
                    count = count + 1
                    print('stop!')
                    print(count)
                    rospy.sleep(3)

                if abs(err) >= 0.5:
                    drive_controller.set_velocity(0.63)
                    drive_controller.set_angular(err)
                    drive_controller.drive()

                elif abs(err) < 0.5:
                    drive_controller.set_velocity(0.63)
                    drive_controller.set_angular(err)
                    drive_controller.drive()

                # change state
                count+=1
                LaneTrace.saved_count = count
                return "success"

            #free drive, fifth stop line (with s course)
            elif count == 5:
                cx = (line.lcx - 40 + line.rcx + 10) / 2 - 330
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

            elif count == 6:
                cx = (line.lcx - 40 + line.rcx + 10) / 2 - 330
                err = -float(cx) / 100
                if line.area > 9000.0:
                    drive_controller.set_velocity(0)
                    drive_controller.set_angular(0)
                    count = count + 1
                    print('stop!')
                    print(count)
                    rospy.sleep(3)

                if abs(err) >= 0.5:
                    drive_controller.set_velocity(0.5)
                    drive_controller.set_angular(err)
                    drive_controller.drive()

                elif abs(err) < 0.5:
                    drive_controller.set_velocity(0.5)
                    drive_controller.set_angular(err)
                    drive_controller.drive()

            #sixth stop line (cross course start)
            elif count == 7:
                cx = (line.lcx - 40 + line.rcx + 10) / 2 - 330
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

                # change state
                count += 1
                LaneTrace.saved_count = count
                return "success"

            elif count == 8:
                cx = (line.lcx - 40 + line.rcx + 10) / 2 - 330
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

            elif count == 9:
                cx = (line.lcx - 40 + line.rcx + 10) / 2 - 330
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
                    # change state
                    count += 1
                    LaneTrace.saved_count = count
                    return "success"

        rospy.sleep()



class Straight(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        drive_controller = RobotDriveController()
        start_time = time.time()+5.8

        print 'go straight'

        while not rospy.is_shutdown():
            drive_controller.set_velocity(1)
            drive_controller.set_angular(0)
            drive_controller.drive()
            if time.time() - start_time > 0:
                break
        return 'success'

class Straight2(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        drive_controller = RobotDriveController()
        start_time = time.time() + 6.8

        print 'go straight'

        while not rospy.is_shutdown():
            drive_controller.set_velocity(1)
            drive_controller.set_angular(0)
            drive_controller.drive()
            if time.time() - start_time > 0:
                break
        return 'success'

class Left(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        drive_controller = RobotDriveController()
        start_time = time.time() + 4.2

        print 'go left'

        while not rospy.is_shutdown():
            drive_controller.set_velocity(0.8)
            drive_controller.set_angular(0.26)
            drive_controller.drive()
            if time.time() - start_time > 0:
                break
        return 'success'


class SCourse(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        line = LineTracer()
        drive_controller = RobotDriveController()
        rospy.Rate(10)
        count = 0

        while not rospy.is_shutdown():
            cx = (line.lcx + line.rcx) / 2 - 340
            err = -float(cx) / 100

            if line.area > 9000.0:
                drive_controller.set_velocity(0)
                drive_controller.set_angular(0)
                count = count + 1
                print('stop!')
                print(count)
                rospy.sleep(3)

            if abs(err) < 0.5:
                drive_controller.set_velocity(0.6)
                drive_controller.set_angular(err)
                drive_controller.drive()
            elif abs(err) >= 0.5:
                drive_controller.set_velocity(0.6)
                drive_controller.set_angular(err)
                drive_controller.drive()
        return 'success'


class AvoidObstacle(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        detect_obstacle = DetectObstacle()
        drive_controller = RobotDriveController()

        # 정면에 없고 오른쪽 대각선 방향에도 없을경우, 각 값이 2 이하일 경우 주행 유지
        if detect_obstacle.range_ahead > 2 or detect_obstacle.range_right > 2 or \
                ((math.isnan(detect_obstacle.range_ahead)) and math.isnan(detect_obstacle.range_right)):
            value = False
            detect_obstacle.stop_pub.publish(value)
            print('go')
        # 아니면 정지 토픽 발행
        else:
            value = True
            detect_obstacle.stop_pub.publish(value)
            drive_controller.set_velocity(0)
            print('stop')

        return 'success'


class DetectStopSign(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        drive_controller = RobotDriveController()
        ic = ImageConverter()
        if ic.match:
            drive_controller.set_velocity(0)
        else:
            pass

        rospy.sleep(3)
        return 'success'


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


