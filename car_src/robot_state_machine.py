#!/usr/bin/env python

import rospy
from smach import StateMachine
from define_robot_state import *

if __name__ == "__main__":
    rospy.init_node('test_node')
    driving_test_site = StateMachine(outcomes=['success'])
    with driving_test_site:
        StateMachine.add('SETTING_LANE', SettingLane(), transitions={'success': 'DETECT_BLOCKING_BAR'})
        StateMachine.add('DETECT_BLOCKING_BAR', DetectBlockingBar(), transitions={'success': 'LANE_TRACE'})
        StateMachine.add('LANE_TRACE', LaneTrace(), transitions={'success': 'ENTER_THE_S_COURSE'})
        StateMachine.add('ENTER_THE_S_COURSE', Straight(), transitions={'success': 'START_S_COURSE_ONE_STEP'})
        StateMachine.add('START_S_COURSE_ONE_STEP', SCourseOneStep(), transitions={'success': 'START_S_COURSE_TWO_STEP'})
        StateMachine.add('START_S_COURSE_TWO_STEP', SCourseTwoStep(), transitions={'success': 'ENTER_THE_T_COURSE'})
        StateMachine.add('ENTER_THE_T_COURSE', Straight2(), transitions={'success': 'success'})
        # StateMachine.add('RIGHT_ANGLE_PARKING', RightAngleParking(), transitions={'success': 'DETECT_OBSTACLE'})
        # StateMachine.add('DETECT_STOP_SIGN1', DetectStopSign(), transitions={'success': 'LANE_TRACE2'})
        # StateMachine.add('LANE_TRACE2', LaneTrace(), transitions={'success': 'DETECT_OBSTACLE'})
        # StateMachine.add('DETECT_OBSTACLE', AvoidObstacle(), transitions={'success': 'LANE_TRACE3'})
        # StateMachine.add('LANE_TRACE3', LaneTrace(), transitions={'success': 'DETECT_STOP_SIGN2'})
        # # StateMachine.add('PARALLEL_PARKING', ParallelParking(), transitions={'success': 'DETECT_STOP_SIGN'})
        # StateMachine.add('DETECT_STOP_SIGN2', DetectStopSign(), transitions={'success': 'success'})
        # StateMachine.add('') ....

    driving_test_site.execute()
    rospy.spin()
