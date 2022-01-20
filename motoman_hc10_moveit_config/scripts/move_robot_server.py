#!/usr/bin/env python
from os import wait
import sys
import rospy
import moveit_commander
from motoman_hc10_moveit_config.srv import Robot_move, Robot_move_predef

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
        print(False)
        return False
    else :
        group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        return True

def handler_robot_move_home(a):
    return move_predef("home")

def handler_robot_move_calibration(a):
    return move_predef("calibration")

def handler_robot_move_localisation(a):
    return move_predef("localisation")

def handler_robot_move_ready(a):
    return move_predef("ready")


def move_robot_server():
    rospy.init_node('move_robot_server', anonymous=True)

    s = rospy.Service('move_robot', Robot_move, handler_robot_move)
    print("Server move robot ready !")

    s1 = rospy.Service('move_robot_home', Robot_move_predef, handler_robot_move_home)
    print("Server move robot to home ready !")

    s2 = rospy.Service('move_robot_calibration', Robot_move_predef, handler_robot_move_calibration)
    print("Server move robot to calibration ready !")

    s3 = rospy.Service('move_robot_localisation', Robot_move_predef, handler_robot_move_localisation)
    print("Server move robot to localisation ready !")

    s4 = rospy.Service('move_robot_ready', Robot_move_predef, handler_robot_move_ready)
    print("Server move robot to ready ready !")

    print("Robot ready to move !")

def move_predef(conf_name):
    target = group.get_named_target_values(conf_name)

    print("Move robot to " + conf_name + ".")
    print("Joint Values " + str(target))

    group.set_joint_value_target(target)
    plan = group.plan()

    if plan.joint_trajectory.joint_names == [] :
        print(False)
        return False
    else :
        group.go(wait=True)
        group.stop()
        return True

if __name__ == "__main__":
    #Limitation de la vitesse
    args = sys.argv[1:]
    if len(args) >= 1:
        speed = 50
        if args[0] == "True" :
            print("speed limited to " + str(speed) + "%.")
            group.set_max_velocity_scaling_factor(speed/100)

    #Lancement des servers
    move_robot_server()
    rospy.spin()