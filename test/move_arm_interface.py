import rospy
import moveit_commander

from enum import Enum
import sys
import PyKDL
from tf_conversions import posemath
from geometry_msgs.msg import PoseStamped
import moveit_msgs.msg


def _pose_stamped_to_kdl_and_frame_id(pose):
    """
    Conversions for ROS Messages
    :param pose: ROS geometry msgs Pose Stamped
    :return: KDL Frame and Frame id
    """
    return posemath.fromMsg(pose.pose), pose.header.frame_id


def _kdl_and_frame_id_to_pose_stamped(pose, frame_id):
    """
    Conversions for KDL Frames: Assumption, take time ros(0) to take latest
    :param pose: KDL Pose
    :param frame_id: Frame id string
    :return: Pose Stamped Message
    """
    msg = PoseStamped()
    msg.pose = posemath.toMsg(pose)
    msg.header.frame_id = frame_id
    return msg


class GoalTypes(Enum):
    """Goal types available for the MoveArmInterface"""
    MOVE = 0
    PRE_GRASP = 1
    PRE_GRASP_GRASP = 2


class MoveArmInterface(object):

    def __init__(self, group_name, pre_grasp_x_offset=-0.1):
        """
        MoveArmInterface to move the arm to grasp something
        :param group_name: The move group, e.g. left_arm, right_arm
        :param pre_grasp_x_offset: How much offset for a pre-grasp goal
        """
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
        """
        Publishes a visualization of a plan
        :param plan: Plan
        """
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self._robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self._visualization_publisher.publish(display_trajectory)

    def _compute_cartesian_plan(self, pose_list, frame_id):
        """
        Computes a cartesian path from a pose list
        :param pose_list: The pose list
        :param frame_id: Frame id string
        :return: The plan
        """
        self._group.set_pose_reference_frame(frame_id)
        path, fraction = self._group.compute_cartesian_path([posemath.toMsg(p) for p in pose_list], 0.01, 10.0, False)
        return path

    @staticmethod
    def _compute_pre_grasp_pose(pose, x_offset):
        """
        Calculate the pose from an x-offset in the pose frame
        :param pose: Initial pose
        :param x_offset: X offset in the pose frame
        :return: The calculated pre grasp pose
        """
        return pose * PyKDL.Frame(PyKDL.Rotation(), PyKDL.Vector(x_offset, 0, 0))

    def _move(self, poses, frame_id, wait):
        """
        Moves the end-effector to a list of poses by computing a cartesian path
        :param poses: Poses list
        :param frame_id: Frame id string
        :param wait: Whether to block while excecuting
        :return: True when success, False otherwise
        """
        plan = self._compute_cartesian_plan(poses, frame_id)

        if not plan:
            rospy.logwarn("Could not find a valid trajectory to specified pose.")
            return False

        rospy.loginfo("Retiming trajectory ..")
        plan = self._group.retime_trajectory(self._robot.get_current_state(), plan, 1.0)

        rospy.loginfo("Visualizing and executing trajectory ..")
        self._visualize_plan(plan)
        return self._group.execute(plan, wait)

    def _move_goal(self, pose, frame_id, wait):
        """
        Callback when a move goal is requested, sends the end-effector to a specific pose
        """
        return self._move([pose], frame_id, wait)

    def _pre_grasp_goal(self, pose, frame_id, wait):
        """
        Callback when a pre-grasp goal is requested, sends the end-effector to a specific x-offset in
        the requested pose frame.
        """
        pre_grasp_pose = self._compute_pre_grasp_pose(pose, self._pre_grasp_x_offset)
        return self._move([pre_grasp_pose], frame_id, wait)

    def _pre_grasp_grasp_goal(self, pose, frame_id, wait):
        """
        Callback when a pre-grasp-grasp goal is requested, sends the end-effector to a specific x-offset in
        the requested pose frame. Afterwards, it will approach the requested pose.
        """
        pre_grasp_pose = self._compute_pre_grasp_pose(pose, self._pre_grasp_x_offset)
        return self._move([pre_grasp_pose, pose], frame_id, wait)

    def send_goal(self, pose, frame_id, goal_type=GoalTypes.MOVE, wait=True):
        """
        Public interface to send a goal to the MoveIt interface, calls the different goal type callback
        :param pose: KDL Pose
        :param frame_id: Frame id string
        :param goal_type: Goal type enum
        :param wait: Whether to block when executing the trajectory
        :return: True if succeeded
        """
        rospy.loginfo("Received goal of type %s in frame %s: \n%s", goal_type, frame_id, pose)
        if goal_type not in self._goal_methods:
            rospy.logerr("Received invalid goal type %s, please provide a valid goal type!", goal_type)
            return False
        return self._goal_methods[goal_type](pose, frame_id, wait)
