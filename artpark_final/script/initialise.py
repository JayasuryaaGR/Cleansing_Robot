#!/usr/bin/env python3

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):

    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroup(object):

    def __init__(self):
        super(MoveGroup, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_joint_state(self):

        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = -1.5708
        joint_goal[1] = -0.5
        joint_goal[2] = 0 # nasty
        joint_goal[3] = -0.8
        joint_goal[4] = 0 # nasty
        joint_goal[5] = 1  # 1/6 of a turn
        joint_goal[6] = 0

        # joint_goal[0] = -1.5708
        # joint_goal[1] = -0.5
        # joint_goal[2] = 0 # nasty
        # joint_goal[3] = -0.5
        # joint_goal[4] = 0 # nasty
        # joint_goal[5] = 0  # 1/6 of a turn
        # joint_goal[6] = 0

        # joint_goal = move_group.get_current_joint_values()
        # joint_goal[0] = -1.5708
        # joint_goal[1] = 0
        # joint_goal[2] = 0 # nasty
        # joint_goal[3] = -0.8
        # joint_goal[4] = 0 # nasty
        # joint_goal[5] = 0  # 1/6 of a turn
        # joint_goal[6] = 0

        # joint_goal[0] = 0.0 #-0.24
        # joint_goal[1] = 1.76 #1.76
        # joint_goal[2] = 0.0 #0.0
        # joint_goal[3] = -0.4 #-1.15
        # joint_goal[4] = 0 #0
        # joint_goal[5] = 2.19 #2.19  # 1/6 of a turn
        # joint_goal[6] = 0.0

        move_group.go(joint_goal, wait=True)
        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    

def main():

    rospy.init_node("initialise")

    print("=========== Robot and Scene Initialising, Please Wait! ===========")
    time.sleep(15)
    tutorial = MoveGroup()
    tutorial.go_to_joint_state()
    print("============ Robot And Scene Initialised! ===========")


if __name__ == "__main__":
    main()
