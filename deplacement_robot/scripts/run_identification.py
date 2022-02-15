#!/usr/bin/env python
from StepReader import StepReader
from useful_robot import rotation_between_vect, homogeneous_matrix_to_pose_msg, get_fk, pose_msg_to_homogeneous_matrix
import numpy as np
import rospy
import rospkg
from deplacement_robot.srv import Robot_move, Robot_move_predef, Robot_set_state
from communication.srv import identification
from deplacement_robot.msg import Identification, Trou_identification
from geometry_msgs.msg import Pose

from cv_bridge import CvBridge

import matplotlib.pyplot as plt

def run_identification(plaque_pos, nom_plaque, step_folder, diametres, dist =  1.03, seuil=100, pub = None):
    pub_state(pub, "Debut identification")
    pub_state(pub, "Calcul de la trajectoire")

    # Lecture du fichier step et recuperation des trous
    cylinders_dict = get_cylinders(step_folder + "/" + str(nom_plaque) + ".stp")

    # Determination du chemin en fonction des trous
    poses = get_poses(cylinders_dict, plaque_pos, dist)

    pub_state(pub, "Trajectoire trouvee")

    ######## Deplacement du robot ########
    # Service pour deplacer le robot a un point donne
    move_robot = rospy.ServiceProxy('move_robot', Robot_move)
    # Service pour deplacer le robot a sa position de parking
    move_parking = rospy.ServiceProxy('move_robot_parking', Robot_move_predef)
    # Service pour effectuer l identification
    identification_srv = rospy.ServiceProxy("camera/identification", identification)

    # Dictionnaire point 3D (x,y,z) : diametre
    dict_point = {}

    res_points = []
    res_image_originale = []
    res_image_annotee = []
    decalage = 300
    
    bridge = CvBridge()

    # Determination du type de plaque pour savoir si on doit faire des bande en image
    type_plaque = "Plate"

    if len(poses) > 1:
        type_plaque = "Courbee"

    # Deroulement de la trajectoire et de l'identification
    nbPose = len(poses)
    for i,pose in enumerate(poses):
        if rospy.is_shutdown():
            exit()
        pub_state(pub, "Deplacement a la position " + str(i+1) + "/" + str(nbPose))
        move_parking()
        res_move = move_robot(pose)

        if not res_move.res :
            pub_state(pub, "Position inatteignable. Passage au position suivante")
            rospy.logwarn("Point inatteignable. Passage au point suivant.")
            res_points.append([])
            res_image_originale.append(np.zeros((1944,decalage,3), dtype=np.uint8))
            res_image_annotee.append(np.zeros((1944,decalage,3), dtype=np.uint8))
        else:
            pub_state(pub, "Identification des trous")
            res_identification = identification_srv(diametres, str(type_plaque))
            res_points.append(res_identification.points.points)
            res_image_originale.append(bridge.imgmsg_to_cv2(res_identification.originale, 'bgr8'))
            res_image_annotee.append(bridge.imgmsg_to_cv2(res_identification.annotee, 'bgr8'))
    
    pub_state(pub, "Identification finie, retour position parking")

    move_parking()

    image_annotee = np.concatenate(res_image_annotee,axis=1)
    image_originale = np.concatenate(res_image_originale,axis=1)

    points_msg = []
    trous = []

    pub_state(pub, "Traitement des donnees")

    for i,points in enumerate(res_points):
        for point in points:
            if rospy.is_shutdown():
                exit()

            p = Trou_identification()
            p.x = point.x
            p.y = point.y + i * decalage
            p.diam = point.type

            trous.append((p.x,p.y))
            points_msg.append(p)


    msg = Identification()
    msg.trous = points_msg
    msg.nbTrous = len(points_msg)

    msg.image = bridge.cv2_to_compressed_imgmsg(image_annotee)

    plt.imshow(image_annotee)
    plt.draw()
    plt.pause(0.001)
    '''print("press enter")
    raw_input()'''
    pub_state(pub, "Identification terminee.")

    return msg, (image_originale, trous)

def pub_state(pub, msg):
    if not pub is None:
        pub.publish(msg)

def get_points_projection(intrinsic, extrinsic, P0):
    # [xi,yi,1] = 1/z*[intrinseque].[extrinseque].[point]
    P0 = np.hstack((P0, 1))
    Pc = np.dot(extrinsic,P0)
    Pi = 1/Pc[2]*np.dot(intrinsic, Pc[:3])
    pass

def get_poses(cylinders_dict, plaque_pos, dist):
    poses = []
    poses_d = {}

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

        pt = np.hstack((r, p.T))
        pt = np.vstack((pt, [0,0,0,1]))
        msg = homogeneous_matrix_to_pose_msg(pt)

        poses_d[tuple(p[0])] = msg
        
    keys = poses_d.keys()
    for key in sorted(list(keys)):
        poses.append(poses_d[key])

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
    run_identification(R, "Plaque_1", cwd + "/plaques", [5,7,12,18])
