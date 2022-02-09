#!/usr/bin/env python
from StepReader import StepReader
from useful_robot import rotation_between_vect, homogeneous_matrix_to_pose_msg, get_fk, pose_msg_to_homogeneous_matrix
import numpy as np
import rospy
import rospkg
from deplacement_robot.srv import Robot_move, Robot_move_predef, Robot_set_state
from communication.srv import identification
from deplacement_robot.msg import Identification, Trou_identification

from  geometry_msgs.msg import Pose

# TODO Fusion de donnees
# TODO Traiter que les points vouluent
# TODO Appel du service
# TODO Message ROS

def run_identification(plaque_pos, nom_plaque, step_folder, diametres, global_picture, intrinsic=np.eye(3), dist =  0.93, seuil=100):
    # Lecture du fichier step et recuperation des trous
    cylinders_dict = get_cylinders(step_folder + "/" + str(nom_plaque) + ".stp")

    # Determination du chemin en fonction des trous
    poses = get_poses(cylinders_dict, plaque_pos, dist)

    ######## Deplacement du robot ########
    # Service pour deplacer le robot a un point donne
    move_robot = rospy.ServiceProxy('move_robot', Robot_move)
    # Service pour deplacer le robot a sa position de parcking
    move_parcking = rospy.ServiceProxy('move_robot_parcking', Robot_move_predef)
    # Service pour effectuer l identification
    identification_srv = rospy.ServiceProxy("camera/identification", identification)

    # Dictionnaire point 3D (x,y,z) : diametre
    dict_point = {}

    move_parcking()

    result_msg = Identification()
    res_points = []

    # DEBUG
    nbTrousTraite = 0

    for pose in poses:
        if rospy.is_shutdown():
            exit()
        
        resp1 = move_robot(pose[0])

        # Récupération de la postion de la caméra
        pos_cam = pose_msg_to_homogeneous_matrix(get_fk())
        # Point devant être pris en compte (2D, 3D)
        points = []

        for point in pose[1]:
            # Passage du point dans le repère image
            p = get_points_projection(intrinseques, extrinseques, point)
            points.append[(p, point)]

        res_identification = identification_srv()

        for point in res_identification.points.points:
            if rospy.is_shutdown():
                exit()

            # On traite que les trous qui ont le bon diametre
            if point.type in diametres :
                Pi = np.array([point.x, point.y])
                for p in points:
                    # On verifie si le trous est bien un de ceux dont on est en face
                    Pc = np.array(p[0])
                    if np.linalg.norm(Pi-Pc) <= seuil:
                        dict_point[tuple(p[1])] = point.type
                        # DEBUG
                        nbTrousTraite += 1
                        pr = Trou_identification()
                        pr.x = p[1][0]
                        pr.x = p[1][1]
                        pr.diametre = point.type
                        res_points.append(pr)
                        break

        print("press enter")
        raw_input()

    if rospy.is_shutdown():
        exit()

    # DEBUG
    print("nbTrousTraite : " + str(nbTrousTraite))

    if rospy.is_shutdown():
        exit()

    move_parcking()

    return result_msg

def get_points_projection(intrinsic, extrinsic, P0):
    # [xi,yi,1] = 1/z*[intrinseque].[extrinseque].[point]
    P0 = np.hstack((P0, 1))
    Pc = np.dot(extrinsic,P0)
    Pi = 1/Pc[2]*np.dot(intrinsic, Pc[:3])
    pass

def get_poses(cylinders_dict, plaque_pos, dist):
    poses = []

    for key in cylinders_dict:
        cylinders = cylinders_dict[key]
        point = []
        for c in cylinders :
            point.append(c.position)

        m = np.mean(point, 0)/1000
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
        poses.append((msg, point))

    return poses

def get_cylinders(file_path):
    step = StepReader(file_path)
    cylinders_dict = {}
    
    for c in step.getCylinders():
        if c.rayon <= 10:
            l = cylinders_dict.get(c.direction, [])
            l.append(c)
            cylinders_dict[c.direction] = l

    return cylinders_dict

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
    cwd = rospack.get_path("deplacement_robot")
    run_identification(R, "Plaque_1", cwd + "/plaques")