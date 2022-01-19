#!/usr/bin/env python
from os import wait
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from motoman_hc10_moveit_config.srv import Robot_move, Robot_move_predef
import numpy as np

moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator")

def handler_robot_move(msg):
    pose_goal = msg.Pose
    print("Move robot to : " + str(pose_goal))

    group.set_pose_target(pose_goal)

    plan = group.plan()

    if plan.joint_trajectory.joint_names == [] :
        print(false)
        return False
    else :
        group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        return True

def handler_robot_move_home(a):
    target = group.get_named_target_values("home")

    print("Move robot to home.")
    print("Joint Values " + str(target))

    group.set_joint_value_target(target)
    plan = group.plan()

    if plan.joint_trajectory.joint_names == [] :
        print(false)
        return False
    else :
        group.go(wait=True)
        group.stop()
        return True



def move_robot_server():
    rospy.init_node('move_robot_server', anonymous=True)

    s = rospy.Service('move_robot', Robot_move, handler_robot_move)
    print("Server move robot ready !")

    s1 = rospy.Service('move_robot_home', Robot_move_predef, handler_robot_move_home)
    print("Server move robot to home ready !")

    print("Robot ready to move !")

if __name__ == "__main__":
    #Limitation de la vitesse
    args = sys.argv[1:]
    if len(args) >= 1:
        if args[0] == "True" :
            print("Limited speed.")
            group.set_max_velocity_scaling_factor(0.01)

    #Lancement des servers
    move_robot_server()
    rospy.spin()