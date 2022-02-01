#!/usr/bin/env python

from StepReader import StepReader
from useful_robot import rotation_between_vect, homogeneous_matrix_to_pose_msg
import numpy as np
import rospy
from motoman_hc10_moveit_config.srv import Robot_move, Robot_move_predef

def run_identification(plaque_pos, nom_plaque, step_folder, dist =  0.7):
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

        if p2[2] > p1[2]:
            r = rotation_between_vect((0,0,1), m-p2[:3])
            if (r == -np.eye(3)).all():
                r[0,0] = 1
            p = np.array([p2[:3]])
            p = np.hstack((r, p.T))
            p = np.vstack((p, [0,0,0,1]))
            points.append(p)

        else :
            r = rotation_between_vect((0,0,1), m-p1[:3])
            if (r == -np.eye(3)).all():
                r[0,0] = 1
            p = np.array([p1[:3]])
            p = np.hstack((r, p.T))
            p = np.vstack((p, [0,0,0,1]))
            points.append(p)

    move_robot = rospy.ServiceProxy('move_robot_lin', Robot_move)
    move_parcking = rospy.ServiceProxy('move_robot_parcking', Robot_move_predef)

    for p in points:
        print(p)
        msg = homogeneous_matrix_to_pose_msg(p)
        
        resp0 = move_parcking()
        resp1 = move_robot(msg)
        print("press enter")
        raw_input()



if __name__ == "__main__":
    rospy.init_node('test_identification', anonymous=True)
    R = np.eye(4)
    R[0,3] = 0.5
    R[1,3] = 0.5
    R[2,3] = -0.270 + 0.275
    run_identification(R, "Plaque_2", "/home/alexandre")