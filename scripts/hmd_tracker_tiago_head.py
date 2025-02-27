#!/usr/bin/env python3

from cmath import pi
from curses import noecho
from locale import normalize

from matplotlib import rc
from rosgraph.names import GLOBALNS
import rospy
import time
import control_msgs.msg 
# TF stuff
from geometry_msgs.msg import PoseStamped, Pose, Point
from control_msgs.msg import JointTrajectoryControllerState
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from math import radians
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf
from std_msgs.msg import String, Bool, Int16, Int32, Float32
from pyquaternion import Quaternion, quaternion
from gazebo_msgs.msg import LinkStates
import math
import numpy as np
from std_srvs.srv import Empty
from visualization_msgs.msg import MarkerArray, Marker
from actionlib import SimpleActionClient, GoalStatus
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import Joy,JoyFeedback


control_rate = 30

class TiagoTeleopHead(object):
    def __init__(self):
        ####### Publishers #######
        self.init_pub = rospy.Publisher('/state_center/init_pose',Pose, queue_size=1)
        self.head_roll_pub = rospy.Publisher('/head_real/roll', Float32, queue_size=1)

        ####### Subscribers #######
        self.hmd_command = rospy.Publisher('/head_controller/command',JointTrajectory,queue_size=1)
        self.hmd_pitch_sub = rospy.Subscriber('/head/pitch', Float32, self.hmd_pitch_cb, queue_size=1)
        self.hmd_pitch = 0.0
        self.hmd_yaw_sub = rospy.Subscriber('/head/yaw', Float32, self.hmd_yaw_cb, queue_size=1)
        self.hmd_yaw = 0.0

        # soft flag
        self.first_time_flag = True

    def hmd_pitch_cb(self, msg):
        self.hmd_pitch = msg.data

    def hmd_yaw_cb(self, msg):
        self.hmd_yaw = msg.data

    def send_hmd_goal(self, positions):
        # # TODO >>>
        # print(positions)
        # des_joint_states = [np.array(positions)]
        # times_from_start = rospy.Duration.from_sec(3)
        # times_from_start = [times_from_start * (i+1) for i in range(len(des_joint_states))]
        
        # des_joint_vels = [np.zeros(2)] * len(des_joint_states)
        # print(des_joint_states)
        # print(des_joint_vels)
            
        # print(times_from_start)
        # points = []
        # for joint_state, joint_vel, t in zip(des_joint_states, des_joint_vels, times_from_start):
        #     points.append(JointTrajectoryPoint(positions=joint_state, velocities=joint_vel, time_from_start=t))
        # print(points)
        # # # TODO <<<
        
        
        jt = JointTrajectory()
        # jt.header.stamp = rospy.Time.now()
        jt.joint_names = ["head_1_joint", "head_2_joint"]
        jtp = JointTrajectoryPoint()
        jtp.positions = list(positions)
        # jtp.velocities = [0.0] * len(positions)
        jtp.time_from_start = rospy.Time(1)   #default 0.4
        jt.points.append(jtp)
        self.hmd_command.publish(jt)  

    def init_robot_pose(self):
        # Init head position
        self.send_hmd_goal([0, 0])

        rospy.loginfo('...done')
        rospy.sleep(rospy.Duration(5))  
        
    def run_with_ik(self):
        self.init_robot_pose()
        rospy.loginfo('...start')
        r = rospy.Rate(30) #default 4
        rospy.sleep(0.5)
        while not rospy.is_shutdown():
            if self.first_time_flag:
                self.first_time_flag = False
                ### init the head pose
                # not use last_hmd_pose again
                [start_pitch, start_yaw] = [self.hmd_pitch, self.hmd_yaw]

            ### head control
            # only pitch and yaw is useful
            [pitch, yaw] = [self.hmd_pitch, self.hmd_yaw]
            goal_pitch = (pitch - start_pitch) - 0.4
            goal_yaw = -(yaw - start_yaw)
            if(goal_pitch<=-1): goal_pitch= -1
            elif(goal_pitch>=1): goal_pitch= 1
            if(goal_yaw<=-1.2): goal_yaw = -1.3
            elif(goal_yaw>1.2): goal_yaw = 1.3
            self.send_hmd_goal([goal_yaw, goal_pitch])
            r.sleep()


if __name__ == '__main__':
    rospy.init_node('tiago_pure_head')
    teleop_head = TiagoTeleopHead()
    teleop_head.run_with_ik()

