#!/usr/bin/env python
from StepReader import StepReader
from useful_robot import rotation_between_vect, homogeneous_matrix_to_pose_msg
import numpy as np
import rospy
import rospkg
from motoman_hc10_moveit_config.srv import Robot_move, Robot_move_predef

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

        msg = Pose()

        print(p)

        # Extracting the translation
        msg.position.x = p[0,0]
        msg.position.y = p[0,1]
        msg.position.z = p[0,2]

        # Conversion of the rotation matrix into a quaternion
        v = np.cross(v,[1,0,0])
        msg.orientation.x = v[0]
        msg.orientation.y = v[1]
        msg.orientation.z = v[2]
        msg.orientation.w = 0
        
        points.append(msg)

        '''
        r = rotation_between_vect((0,0,0), v)
        #r = get_orientation_mat(v)
        if (r == -np.eye(3)).all():
            r[0,0] = 1

        p = np.hstack((r, p.T))
        p = np.vstack((p, [0,0,0,1]))
        points.append(p)

        print("m:")
        print(m)
        print("v:")
        print(v)
        print("p:")
        print(p)'''

    move_robot = rospy.ServiceProxy('move_robot_lin', Robot_move)
    move_parcking = rospy.ServiceProxy('move_robot_parcking', Robot_move_predef)

    for p in points:
        print(p)
        #msg = homogeneous_matrix_to_pose_msg(p)
        
        resp0 = move_parcking()
        resp1 = move_robot(p)
        print("press enter")
        raw_input()

def get_orientation_mat(tz) :
    tz = tz / np.linalg.norm(tz)
    tx = np.cross(tz,[0,1,0])
    tx = tx / np.linalg.norm(tx)
    ty = np.cross(tz, tx)
    return np.array([tx,ty,tz])

if __name__ == "__main__":
    rospy.init_node('test_identification', anonymous=True)
    R = np.eye(4)
    R[0,3] = 0.5
    R[1,3] = 0.2
    R[2,3] = -0.270 + 0.275
    rospack = rospkg.RosPack()
    cwd = rospack.get_path("motoman_hc10_moveit_config")
    run_identification(R, "Plaque_2", cwd + "/plaques")