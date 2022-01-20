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
    s = rospy.Service('move_robot', Robot_move, handler_robot_move)
    rospy.loginfo("Server move robot ready !")

    s1 = rospy.Service('move_robot_home', Robot_move_predef, handler_robot_move_home)
    rospy.loginfo("Server move robot to home ready !")

    s2 = rospy.Service('move_robot_calibration', Robot_move_predef, handler_robot_move_calibration)
    rospy.loginfo("Server move robot to calibration ready !")

    s3 = rospy.Service('move_robot_localisation', Robot_move_predef, handler_robot_move_localisation)
    rospy.loginfo("Server move robot to localisation ready !")

    s4 = rospy.Service('move_robot_ready', Robot_move_predef, handler_robot_move_ready)
    rospy.loginfo("Server move robot to ready ready !")

    rospy.loginfo("Robot ready to move !")

def move_predef(conf_name):
    target = group.get_named_target_values(conf_name)

    rospy.loginfo("Move robot to " + conf_name + ".")
    rospy.loginfo("Joint Values " + str(target))

    group.set_joint_value_target(target)
    plan = group.plan()

    if plan.joint_trajectory.joint_names == [] :
        rospy.logerr("Unreachable position")
        return False
    else :
        group.go(wait=True)
        group.stop()
        return True

if __name__ == "__main__":
    rospy.init_node('move_robot_server', anonymous=True)

    #Limitation de la vitesse    
    args = sys.argv[1:]
    if len(args) >= 1:
        try:
            speed = float(args[0])
            group.set_max_velocity_scaling_factor(speed/100)
            rospy.loginfo("Speed limited to " + str(speed) + "%.")
        except ValueError:
            rospy.logerr('Error the speed percentage must be float ! No : "' + args[0] + '"')
            rospy.signal_shutdown("Error speed percentage value.")
            
    #Lancement des servers
    if not rospy.is_shutdown():
        move_robot_server()
        rospy.spin()