#!/usr/bin/env python
from http.client import CONTINUE
from StepReader import StepReader
from useful_robot import rotation_between_vect, homogeneous_matrix_to_pose_msg
import numpy as np
import rospy
import rospkg
from motoman_hc10_moveit_config.srv import Robot_move, Robot_move_predef, Robot_set_state

from  geometry_msgs.msg import Pose

def run_qualite(plaque_pos, nom_plaque, step_folder, dist =  0.2):
    step = StepReader(step_folder + "/" + str(nom_plaque) + ".stp")

    points = []
    d = {}
    
    for c in step.getCylinders():
        if c.rayon <= 10:
            d[c.position] = c.direction
            
    keys = sorted(list(d.keys()))

    for m in keys :
        v = np.array(d[m])
        m = np.array(m)/1000
        m = np.dot(plaque_pos, np.hstack((m, 1)))[:3]

        p1 = m + dist * v
        p2 = m - dist * v

        p = None

        if p2[2] > p1[2]:
            p = np.array([p2[:3]])
        else :
            v= -v
            p = np.array([p1[:3]])

        print(p)

        R = get_orientation_mat(v)

        h = np.hstack((R,p.T))
        h = np.vstack((h, [0,0,0,1]))

        points.append(homogeneous_matrix_to_pose_msg(h))

    move_robot = rospy.ServiceProxy('move_robot', Robot_move)
    move_parcking = rospy.ServiceProxy('move_robot_parcking', Robot_move_predef)
    # Service pour la release 4 uniquement
    set_state = rospy.ServiceProxy("set_robot_state", Robot_set_state)

    set_state("EN PRODUCTION")
    move_parcking()

    for p in points:
        print(p)

        if rospy.is_shutdown():
            set_state("LIBRE NON INIT")
            exit()
        
        resp1 = move_robot(p)
        print("press enter")
        raw_input()

    if rospy.is_shutdown():
        set_state("LIBRE NON INIT")
        exit()

    move_parcking()
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
    R[0,3] = 0.55
    R[1,3] = 0.24
    R[2,3] = -0.270 + 0.275
    rospack = rospkg.RosPack()
    cwd = rospack.get_path("motoman_hc10_moveit_config")
    run_qualite(R, "Plaque_1", cwd + "/plaques")