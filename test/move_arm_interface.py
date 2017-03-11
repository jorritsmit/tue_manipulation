import rospy
import moveit_commander

from enum import Enum
import sys
import PyKDL
from tf_conversions import posemath
from geometry_msgs.msg import PoseStamped
import moveit_msgs.msg


def _pose_stamped_to_kdl_and_frame_id(pose):
    return posemath.fromMsg(pose.pose), pose.header.frame_id


def _kdl_and_frame_id_to_pose_stamped(pose, frame_id):
    """
    Assumption, take time ros(0) to take latest
    :param pose:
    :param frame_id:
    :return:
    """
    msg = PoseStamped()
    msg.pose = posemath.toMsg(pose)
    msg.header.frame_id = frame_id
    return msg


class GoalTypes(Enum):
    MOVE = 0
    PRE_GRASP = 1
    PRE_GRASP_GRASP = 2


class MoveArmInterface(object):

    def __init__(self, group_name, pre_grasp_x_offset=-0.1):

        moveit_commander.roscpp_initialize(sys.argv)

        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(group_name)

        self._goal_methods = {
            GoalTypes.MOVE: self._move_goal,
            GoalTypes.PRE_GRASP: self._pre_grasp_goal,
            GoalTypes.PRE_GRASP_GRASP: self._pre_grasp_grasp_goal
        }

        self._visualization_publisher = rospy.Publisher('move_arm_trajectory',
                                                        moveit_msgs.msg.DisplayTrajectory,
                                                        queue_size=1)

        if pre_grasp_x_offset >= 0:
            rospy.logwarn("You have specified a positive x_grasp offset of %.2f", pre_grasp_x_offset)

        self._pre_grasp_x_offset = pre_grasp_x_offset

    def _visualize_plan(self, plan):
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self._robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self._visualization_publisher.publish(display_trajectory)

    def _compute_cartesian_plan(self, pose_list, frame_id):
        self._group.set_pose_reference_frame(frame_id)
        path, fraction = self._group.compute_cartesian_path([posemath.toMsg(p) for p in pose_list], 0.01, 10.0, False)
        return path

    @staticmethod
    def _compute_pre_grasp_pose(pose, x_offset):
        return pose * PyKDL.Frame(PyKDL.Rotation(), PyKDL.Vector(x_offset, 0, 0))

    def _move(self, poses, frame_id, wait):
        plan = self._compute_cartesian_plan(poses, frame_id)

        if not plan:
            rospy.logwarn("Could not find a valid trajectory to specified pose.")
            return False

        rospy.loginfo("Retiming trajectory ..")
        #plan = self._group.retime_trajectory(self._robot.get_current_state(), plan, 1.0)
        print plan

        rospy.loginfo("Visualizing and executing trajectory ..")
        self._visualize_plan(plan)
        return self._group.execute(plan, wait)

    def _move_goal(self, pose, frame_id, wait):
        return self._move([pose], frame_id, wait)

    def _pre_grasp_goal(self, pose, frame_id, wait):
        pre_grasp_pose = self._compute_pre_grasp_pose(pose, self._pre_grasp_x_offset)
        return self._move([pre_grasp_pose], frame_id, wait)

    def _pre_grasp_grasp_goal(self, pose, frame_id, wait):
        pre_grasp_pose = self._compute_pre_grasp_pose(pose, self._pre_grasp_x_offset)
        return self._move([pre_grasp_pose, pose], frame_id, wait)

    def send_goal(self, pose, frame_id, goal_type=GoalTypes.MOVE, wait=True):
        rospy.loginfo("Received goal of type %s in frame %s: \n%s", goal_type, frame_id, pose)
        if goal_type not in self._goal_methods:
            rospy.logerr("Received invalid goal type %s, please provide a valid goal type!", goal_type)
            return False
        return self._goal_methods[goal_type](pose, frame_id, wait)
