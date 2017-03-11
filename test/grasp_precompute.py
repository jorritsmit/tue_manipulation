#!/usr/bin/env python

from move_arm_interface import MoveArmInterface, GoalTypes

import rospy
import PyKDL

rospy.init_node("grasp_precompute")

arm = MoveArmInterface("left_arm")

arm.send_goal(PyKDL.Frame(PyKDL.Rotation(), PyKDL.Vector(0.5, 0.3, 0.5)), "amigo/base_link", GoalTypes.MOVE)
arm.send_goal(PyKDL.Frame(PyKDL.Rotation(), PyKDL.Vector(0.5, 0.3, 0.8)), "amigo/base_link", GoalTypes.PRE_GRASP_GRASP)
arm.send_goal(PyKDL.Frame(PyKDL.Rotation(), PyKDL.Vector(0.5, 0.3, 0.8)), "amigo/base_link", GoalTypes.PRE_GRASP)

rospy.sleep(2.0)
