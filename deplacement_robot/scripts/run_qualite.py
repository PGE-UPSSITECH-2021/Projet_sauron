#!/usr/bin/env python
from http.client import CONTINUE
from StepReader import StepReader
from useful_robot import rotation_between_vect, homogeneous_matrix_to_pose_msg
import numpy as np
import rospy
import rospkg
from deplacement_robot.srv import Robot_move, Robot_move_predef
from communication.srv import capture
from qualite_interface import fonction_qualite,run_qualite_image_globale

from  geometry_msgs.msg import Pose
from deplacement_robot.msg import Qualite, Trou_qualite
from cv_bridge import CvBridge

from tsp_solver.greedy import solve_tsp


def run_qualite(plaque_pos, nom_plaque, step_folder, dist =  0.18, diametres = [5,7,12,18], pub=None):
    pub_state(pub, "Debut conformite")
    pub_state(pub, "Calcul de la trajectoire")
    # Lecture du fichier step et recuperation de tous les trous
    d = get_holes(step_folder + "/" + str(nom_plaque) + ".stp", diametres)

    # Determination du chemin en fonction des trous
    points = get_path(plaque_pos, dist, d)

    pub_state(pub, "Trajectoire trouvee")

    ######## Deplacement du robot ########

    # Service pour deplacer le robot a un point donne
    move_robot = rospy.ServiceProxy('move_robot', Robot_move)
    # Service pour deplacer le robot a sa position de parking
    move_parking = rospy.ServiceProxy('move_robot_parking', Robot_move_predef)
    # Service pour prendre une image
    capture_image = rospy.ServiceProxy("camera/capture", capture)

    pub_state(pub, "Deplacement a la position de parking")
    move_parking()
    pub_state(pub, "Deplacement termine")

    bridge = CvBridge()
    returned_msg = Qualite()
    trous=[]
    results_qualite = {} # results_qualite = {diametre:{(x,y,z):(msg_ROS,conforme)}}

    nbPose = len(points)
    isDefective_all = []
    # Effectuer la trajectoire
    for i,p in enumerate(points):
        # On arrete si le node est kill
        if rospy.is_shutdown():
            exit()
        
        pub_state(pub, "Deplacement a la position " + str(i+1) + "/" + str(nbPose))
        # Le robot se deplace au point p
        resp1 = move_robot(p[0])

        if not resp1:
            # Si le deplacement echou on passe au point suivant
            pub_state(pub, "Position inatteignable. Passage au position suivante")
            rospy.logwarn("Point inatteignable. Passage au point suivant.")
            continue

        pub_state(pub, "Deplacement termine")

        trou_qualite_msg = Trou_qualite()

        pub_state(pub, "Prise d'image")

        res = capture_image()

        pub_state(pub, "Verification de la conformite")

        rosimage = res.image
        cv_image = bridge.imgmsg_to_cv2(rosimage, 'bgr8')

        print("shape = ",cv_image.shape)
        assert (len(cv_image.shape) == 3),"(1) probleme dimensions, image BGR ?"

        isdefective, defect, image = fonction_qualite(p[1],cv_image,debug=False,fast_algo=True)
        isDefective_all.append(isdefective)

        image_ros_result = bridge.cv2_to_compressed_imgmsg(image)

        trou_qualite_msg.x= p[2][0]
        trou_qualite_msg.y= p[2][1]
        trou_qualite_msg.diam= p[1]
        trou_qualite_msg.conforme   = isdefective
        trou_qualite_msg.raison     = defect
        trou_qualite_msg.image      = image_ros_result

        trous.append(trou_qualite_msg)

        dict_res_qual = results_qualite.get(p[1]*2,{})
        dict_res_qual[tuple(p[2])] = (p[0],isdefective)
        results_qualite[p[1]*2] = dict_res_qual

        # Pour les tests
        #print("Print diametre pour test : ",trou_qualite_msg.diam)
        #print("press enter")
        #raw_input()
    
    # TODO
    #Fonction image globale qualite
    #run_qualite_image_globale(liste_points_px,image_globale,isDefective_all)

    #returned_msg.image=None #TODO
    returned_msg.trous = trous
    returned_msg.nbTrous = len(trous)

    if rospy.is_shutdown():
        exit()

    pub_state(pub, "Conformite finie, retour au parking")

    # Retour a la position de parking
    move_parking()

    pub_state(pub, "Conformite terminee.")

    return returned_msg, results_qualite


def pub_state(pub, msg):
    if not pub is None:
        pub.publish(msg)

def get_holes(file_path, diametres):
    step = StepReader(file_path)

    d = {}
    
    for c in step.getCylinders():
        if c.rayon <= 10 and c.rayon*2 in diametres:
            d[c.position] = c

    return d

def get_path(plaque_pos, dist, d):
    points = []
    
    keys = d.keys()

    dist_keys = get_distance(keys)
    order = solve_tsp(dist_keys)

    for o in order :
        k = keys[o]
        v = np.array(d[k].direction)
        m = np.array(k)/1000
        m = np.dot(plaque_pos, np.hstack((m, 1)))[:3]

        p1 = m + dist * v
        p2 = m - dist * v

        p = None

        if p2[2] > p1[2]:
            p = np.array([p2[:3]])
        else :
            v= -v
            p = np.array([p1[:3]])

        R = get_orientation_mat(v)

        h = np.hstack((R,p.T))
        h = np.vstack((h, [0,0,0,1]))

        rayon = d[k].rayon
        pos=np.dot(plaque_pos, np.hstack((d[k].position, 1)))[:3]/1000
        msg = homogeneous_matrix_to_pose_msg(h)

        points.append((msg, rayon, pos))

    return points

def get_orientation_mat(tz):
    tz = tz / np.linalg.norm(tz)
    tx = np.cross(tz,[0,1,0])
    tx = tx / np.linalg.norm(tx)
    ty = np.cross(tz, tx)
    ty = ty / np.linalg.norm(ty)

    R = np.hstack((np.array([tx]).T, np.array([ty]).T, np.array([tz]).T))

    return R

def get_distance(v):
    v = np.array(v)
    dist_l = []
    for p in v:
        dist_l.append(np.linalg.norm(v-p,axis=1))
    dist_M = np.array(dist_l)
    dist_M = dist_M.T
    return dist_M

if __name__ == "__main__":
    rospy.init_node('test_identification', anonymous=True)
    R = np.eye(4)
    R[0,3] = 0.55
    R[1,3] = 0.24
    R[2,3] = -0.270 + 0.275
    rospack = rospkg.RosPack()
    cwd = rospack.get_path("deplacement_robot")
    run_qualite(R, "Plaque_1", cwd + "/plaques")
