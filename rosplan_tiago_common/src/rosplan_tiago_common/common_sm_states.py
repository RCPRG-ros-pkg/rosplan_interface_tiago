#!/usr/bin/env python

import rospy
import smach
import smach_ros
import dynamic_reconfigure.client
import actionlib

from move_base_msgs.msg import *
from actionlib_msgs.msg import GoalStatus
from rosplan_tiago_scenarios_msgs.msg import GoWithAttendanceAction
from rosplan_tiago_scenarios_msgs.msg import GoWithAttendanceActionGoal
from rosplan_tiago_scenarios_msgs.msg import GoWithAttendanceActionFeedback
from rosplan_tiago_scenarios_msgs.msg import GoWithAttendanceActionResult
from rosplan_tiago_common.tiago_torso_controller import TiagoSpeechController, TiagoTorsoController, TiagoHeadController
from pal_common_msgs.msg import *

class SetNavParams(smach.State):
    def __init__(self, max_lin_vel, max_lin_accel):
        print "common_sm_states: SetNavParams(" + str(max_lin_vel) + ', ' + str(max_lin_accel) + ")"
        self.max_lin_vel = max_lin_vel
        self.max_lin_accel = max_lin_accel

        self.node_name = "(" + rospy.get_name() + ")"
        
        base_local_planner = rospy.get_param('/move_base/base_local_planner')
        self.local_planner_name = base_local_planner.split('/')[-1]

        self.dynparam_client = dynamic_reconfigure.client.Client('move_base/' + self.local_planner_name)
        smach.State.__init__(self,
                             outcomes=['ok', 'preemption', 'error'])

        # Ensure that we know what we are doing.
        # Check if the given parameters are present in the configuration.
        config = self.dynparam_client.get_configuration()
        if self.local_planner_name == 'PalLocalPlanner':
            assert 'max_vel_x' in config
            assert 'acc_lim_x' in config
        elif self.local_planner_name == 'EBandPlannerROS':
            assert 'max_vel_lin' in config
            assert 'max_acceleration' in config
        elif self.local_planner_name == 'TebLocalPlannerROS':
            assert 'max_vel_x' in config
            assert 'acc_lim_x' in config
        else:
            raise Exception('Local planner "' + self.local_planner_name + '" is not supported.')

    def execute(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(self.node_name, self.__class__.__name__))

        # setting planner params so it the robot moves slowly
        # params = userdata.set_nav_params_params
        # Differrent planners have different parameters
        if self.local_planner_name == 'PalLocalPlanner':
            params = {
                'max_vel_x': self.max_lin_vel,
                'acc_lim_x': self.max_lin_accel
            }
        elif self.local_planner_name == 'EBandPlannerROS':
            params = {
                'max_vel_lin': self.max_lin_vel,
                'max_acceleration': self.max_lin_accel
            }
        elif self.local_planner_name == 'TebLocalPlannerROS':
            params = {
                'max_vel_x': self.max_lin_vel,
                'acc_lim_x': self.max_lin_accel
            }
        else:
            raise Exception('Local planner "' + self.local_planner_name + '" is not supported.')

        config = self.dynparam_client.update_configuration(params)

        return 'ok'


class SetHeight(smach.State):
    def __init__(self, torso_height):
        print "common_sm_states: SetHeight(" + str(torso_height) + ")"
        self.torso_height = torso_height

        self.node_name = "(" + rospy.get_name() + ")"
        self.speech_controller = TiagoSpeechController()

        print "common_sm_states: SetHeight a"

        self.torso_controller = TiagoTorsoController()

        print "common_sm_states: SetHeight b"
        smach.State.__init__(self,
                             outcomes=['ok', 'preemption', 'error'])

    def execute(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(self.node_name, self.__class__.__name__))

        print "common_sm_states: SetHeight.execute"

        self.speech_controller.tts('Setting my height for moving.')

        # set height from 0.00 to 0.35
        self.torso_controller.set_torso_height(self.torso_height)
        rospy.sleep(3)
        print "common_sm_states: SetHeight.execute ok"

        return 'ok'


class ReleaseHead(smach.State):
    def __init__(self, head_action_name):
        self.head_action_name = head_action_name

        self.node_name = "(" + rospy.get_name() + ")"
        self.pal_head_manager_client = actionlib.SimpleActionClient('pal_head_manager/disable', DisableAction)
        if not self.pal_head_manager_client.wait_for_server(timeout=rospy.Duration(5)):
            print "ReleaseHead: action pal_head_manager/disable is not available."
            self.pal_head_manager_client = None

        self.head_controller = TiagoHeadController()
        smach.State.__init__(self,
                             outcomes=['ok', 'preemption', 'error'])

    def execute(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(self.node_name, self.__class__.__name__))

        # bring back default head
        if not self.pal_head_manager_client is None:
            self.pal_head_manager_client.cancel_all_goals()

        # head up
        if self.head_action_name == 'tilt_forward':
            self.head_controller.tilt_forward()

        return 'ok'

