#!/usr/bin/env python
from StepReader import StepReader
from useful_robot import rotation_between_vect, homogeneous_matrix_to_pose_msg
import numpy as np
import rospy
import rospkg
from motoman_hc10_moveit_config.srv import Robot_move, Robot_move_predef, Robot_set_state

from  geometry_msgs.msg import Pose

def run_identification(plaque_pos, nom_plaque, step_folder, dist =  0.93):
    step = StepReader(step_folder + "/" + str(nom_plaque) + ".stp")
    cylinders_dict = {}
    
    for c in step.getCylinders():
        if c.rayon <= 10:
            l = cylinders_dict.get(c.direction, [])
            l.append(c)
            cylinders_dict[c.direction] = l

    points = []

    for key in cylinders_dict:
        cylinders = cylinders_dict[key]
        pos = []
        for c in cylinders :
            pos.append(c.position)

        m = np.mean(pos, 0)/1000
        m = np.dot(plaque_pos, np.hstack((m, 1)))[:3]

        p1 = m + dist * np.array(key)
        p2 = m - dist * np.array(key)

        v = np.array(key)
        p = None

        if p2[2] > p1[2]:
            p = np.array([p2[:3]])
        else :
            v = -v
            p = np.array([p1[:3]])

        r = get_orientation_mat(v)

        p = np.hstack((r, p.T))
        p = np.vstack((p, [0,0,0,1]))
        msg = homogeneous_matrix_to_pose_msg(p)
        points.append(msg)



    move_robot = rospy.ServiceProxy('move_robot', Robot_move)
    move_parcking = rospy.ServiceProxy('move_robot_parcking', Robot_move_predef)
    # Service pour la release 4 uniquement
    set_state = rospy.ServiceProxy("set_robot_state", Robot_set_state)

    set_state("EN PRODUCTION")

    for p in points:
        print(p)

        if rospy.is_shutdown():
            set_state("LIBRE NON INIT")
            exit()
        
        resp1 = move_robot(p)
        print("press enter")
        raw_input()

    set_state("LIBRE INIT")

def get_orientation_mat(tz):
    tz = tz / np.linalg.norm(tz)
    tx = np.cross(tz,[0,1,0])
    tx = tx / np.linalg.norm(tx)
    ty = np.cross(tz, tx)
    ty = ty / np.linalg.norm(ty)

    R = np.hstack((np.array([tx]).T, np.array([ty]).T, np.array([tz]).T))
    print(R)

    return R

if __name__ == "__main__":
    rospy.init_node('test_identification', anonymous=True)
    R = np.eye(4)
    R[0,3] = 0.5
    R[1,3] = 0.2
    R[2,3] = -0.270 + 0.275
    rospack = rospkg.RosPack()
    cwd = rospack.get_path("motoman_hc10_moveit_config")
    run_identification(R, "Plaque_2", cwd + "/plaques")