#!/usr/bin/env python
from os import wait
import sys
import rospy
import moveit_commander
from motoman_hc10_moveit_config.srv import Robot_move, Robot_move_predef, Speed_percentage, Robot_do_square
import useful_robot
import copy



class Move_robot:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        self.used = False #TODO

        #Limitation de la vitesse    
        args = sys.argv[1:]
        if len(args) >= 1:
            try:
                speed = float(args[0])
                self.group.set_max_velocity_scaling_factor(speed/100)
                rospy.loginfo("Speed limited to " + str(speed) + "%.")
            except ValueError:
                rospy.logerr('Error the speed percentage must be float ! No : "' + args[0] + '"')
                rospy.signal_shutdown("Error speed percentage value.")

        #Lancement des servers
        if not rospy.is_shutdown():
            self.move_robot_server()

    def handler_robot_move(self, msg):
        pose_goal = msg.Pose
        print("Move robot to : " + str(pose_goal))

        self.group.set_pose_target(pose_goal)

        plan = self.group.plan()

        if plan.joint_trajectory.joint_names == [] :
            print(False)
            return False
        else :
            self.group.go(wait=True)
            self.group.stop()
            self.group.clear_pose_targets()
            return True

    def handler_robot_move_home(self, a):
        return self.move_predef("home")

    def handler_robot_move_calibration(self, a):
        return self.move_predef("calibration")

    def handler_robot_move_localisation(self, a):
        return self.move_predef("localisation")

    def handler_robot_move_ready(self, a):
        return self.move_predef("ready")

    def handler_get_fk(self, a):
        pose = useful_robot.get_fk()
        print(pose)
        print(useful_robot.pose_msg_to_homogeneous_matrix(pose))
        return True

    def handler_set_speed_perentage(self, msg):
        self.group.set_max_velocity_scaling_factor(msg.speed_percentage/100)
        rospy.loginfo("Speed limited to " + str(msg.speed_percentage) + "%.")
        return 1

    def draw_square(self, a):
        waypoints = []

        wpose = self.group.get_current_pose().pose
        wpose.position.x += 0.3
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.y += 0.3
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.x -= 0.3  
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.y -= 0.3
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
        (plan, fraction) = self.group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        self.group.execute(plan, wait=True)

        return []

    def move_picture(self, a):
        return self.move_predef("picture")

    def move_camera(self, a):
        return self.move_predef("camera")

    def move_robot_server(self):
        s = rospy.Service('move_robot', Robot_move, self.handler_robot_move)
        rospy.loginfo("Server move robot ready !")

        s1 = rospy.Service('move_robot_home', Robot_move_predef, self.handler_robot_move_home)
        rospy.loginfo("Server move robot to home ready !")

        s2 = rospy.Service('move_robot_calibration', Robot_move_predef, self.handler_robot_move_calibration)
        rospy.loginfo("Server move robot to calibration ready !")

        s3 = rospy.Service('move_robot_localisation', Robot_move_predef, self.handler_robot_move_localisation)
        rospy.loginfo("Server move robot to localisation ready !")

        s4 = rospy.Service('move_robot_parcking', Robot_move_predef, self.handler_robot_move_ready)
        rospy.loginfo("Server move robot to parcking ready !")

        s5 = rospy.Service('move_picture', Robot_move_predef, self.move_picture)
        rospy.loginfo("Server move robot to picture ready !")

        s6 = rospy.Service('move_camera', Robot_move_predef, self.move_camera)
        rospy.loginfo("Server move robot to camera ready !")

        s_fk = rospy.Service('get_fk', Robot_move_predef, self.handler_get_fk)

        s_speed = rospy.Service('set_speed_percentage', Speed_percentage, self.handler_set_speed_perentage)

        s_square = rospy.Service('move_square', Robot_do_square, self.draw_square)

        rospy.loginfo("Robot ready to move !")

    def move_predef(self, conf_name):
        target = self.group.get_named_target_values(conf_name)

        rospy.loginfo("Move robot to " + conf_name + ".")
        rospy.loginfo("Joint Values " + str(target))

        self.group.set_joint_value_target(target)
        plan = self.group.plan()

        if plan.joint_trajectory.joint_names == [] :
            rospy.logerr("Unreachable position")
            return False
        else :
            self.group.go(wait=True)
            self.group.stop()
            return True


if __name__ == "__main__":
    rospy.init_node('move_robot_server', anonymous=True)

    moveR = Move_robot()

    if not rospy.is_shutdown():
            rospy.spin()