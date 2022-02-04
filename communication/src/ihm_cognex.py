#!/usr/bin/env python
# coding: utf-8

from Tkinter import *
from PIL import Image
import rospy
import telnetlib
from ftplib import FTP
import time
import cv2 as cv
import numpy as np
from variables_cognex import Variables
from std_msgs.msg import Bool
from communication.msg import Liste_points, Points
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import glob
from communication.srv import capture
from communication.srv import identification
from time import sleep
from re import sub

global variables
variables = Variables()


##############################################
###########       CONNEXION       ############
##############################################

def check_connexion(func):
    def wrap(*args, **kwargs):
        try:
            result = func(*args, **kwargs)
            variables.pub_ok.publish(True)
            return result
        except:
            variables.pub_ok.publish(False)

    return wrap

@check_connexion
def cognex_connect():
    # Creation of telnet connexion
    variables.tn = telnetlib.Telnet(variables.ip, timeout=10)
    variables.tn.write(variables.user+'\r\n') # the user name is admin
    variables.tn.write("\r\n") # there is no password - just return - now logged in
    variables.tn.write("Put Live 1\r\n")
    time.sleep(0.5)
    variables.tn.read_eager()

    # Update of default gain value
    variables.gain = int(read_cognex("SIG003"))

@check_connexion
def talk_telnet(mode):
    # Fonction qui permet de lire les informations sur un terminal Telnet
    val = ''
    buf = ''
    buf = variables.tn.read_eager()
    while (len(buf) == 0):
        buf = variables.tn.read_eager()
    val = buf
    while (len(buf) != 0):
        buf = variables.tn.read_eager()
        val = val + buf
    if (mode == "read") :
        val = val[3:len(val)-2]
        return val

@check_connexion
def read_cognex(case):
    #intialisation des valeurs pour la récupération des informations caméra
    val = ''
    # result = -1
    variables.tn.write(case+"\r\n")
    val = talk_telnet("read")
    #Détection d'une erreur lors de la communication 
    if (val != '#ERR' and val != ''):
        rospy.loginfo(val)
    return val

@check_connexion
def write_cognex(case, val):
    variables.tn.write(case+str(val)+"\r\n")
    talk_telnet("write")

@check_connexion
def get_image():
    # ftp logincognex read image cognex
    variables.ftp = FTP(variables.ip)
    variables.ftp.login(variables.user)

    # download file from cognex
    filename = 'image.bmp'
    lf = open(filename, "wb")
    variables.ftp.retrbinary("RETR " + filename, lf.write)
    lf.close()
    img = cv.imread('image.bmp')

    return img





##############################################
##########       ROS EXCHANGES       #########
##############################################
def detection(msg):
    img = get_image()

    try:
        bridge = CvBridge()
        variables.originale = bridge.cv2_to_imgmsg(img, "bgr8")
    except CvBridgeError as e:
        rospy.loginfo(e)

    variables.pub_originale.publish(variables.originale)
    return variables.originale


def identify(msg):
    img = get_image()

    imgOrg= img.copy()
    imgGray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    imgGray = cv.medianBlur(imgGray, variables.blur)

    circles = cv.HoughCircles(imgGray, cv.HOUGH_GRADIENT, variables.dp, variables.minDist,
                                param1=variables.p1, param2=variables.p2, minRadius=variables.minR, maxRadius=variables.maxR)

    if(not(circles is None)):
        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
	        printCircles(img,i[0],i[1],i[2])
            # cv.circle(img,(i[0],i[1]),i[2],(0,255,0),4)
            # cv.circle(img,(i[0],i[1]),2,(0,0,255),6)

    try:
        bridge = CvBridge()
        variables.originale = bridge.cv2_to_imgmsg(imgOrg, "bgr8")
        variables.annotee = bridge.cv2_to_imgmsg(img, "bgr8")
    except CvBridgeError as e:
        rospy.loginfo(e)

    variables.points = []
    for i in circles[0,:]:
        variables.points.append(Points())
        variables.points[-1].x = i[0]
        variables.points[-1].y = i[1]
        variables.points[-1].type = variables.get_type(i[2])

    variables.pub_originale.publish(variables.originale)
    variables.pub_annotee.publish(variables.annotee)
    variables.pub_points.publish(variables.points)
    return variables.points, variables.originale, variables.annotee






##############################################
##########       GUI INTERFACE       #########
##############################################

def imgNumEvent(val):
    variables.imgNum = val

def blurEvent(val):
    variables.blur = (val//2)*2+1

def dpEvent(val):
    variables.dp = val+1

def minDistEvent(val):
    variables.minDist = val+1
    
def p1Event(val):
    variables.p1 = val+1

def p2Event(val):
    variables.p2 = val+1

def minREvent(val):
    variables.minR = val

def maxREvent(val):
    variables.maxR = val

def changeMethodEvent(val):
    variables.opencv = val

def gainEvent(val):
    variables.gain = val
    write_cognex("SIG003", variables.gain)

def identificationEvent(val):
    variables.identification = False if val==0 else True

def saveEvent(val):
    img = get_image()
    reloadImg(img, save=True)

def printCircles(img,ptx,pty,rayon):
    if (variables.get_type(rayon) == 1) :
        cv.circle(img,(ptx,pty),rayon,(0,255,0),4)

    elif(variables.get_type(rayon) == 2) :
        cv.circle(img,(ptx,pty),rayon,(255,0,0),4)

    elif(variables.get_type(rayon) == 3) :
        cv.circle(img,(ptx,pty),rayon,(255,255,0),4)

    elif(variables.get_type(rayon) == 4) :
        cv.circle(img,(ptx,pty),rayon,(255,0,255),4)

    else:
        cv.circle(img,(ptx,pty),rayon,(0,0,255),4)
        print("rayon erreur :" + str(rayon))

    cv.circle(img,(ptx,pty),2,(0,0,255),6)
	

def reloadImg(img, save=False):
    width = int(img.shape[1] * variables.scale_percent)
    height = int(img.shape[0] * variables.scale_percent)
    if (variables.ip=="192.168.1.100"):
  	    dsize = (width, height)
    else:
	    dsize = (img.shape[1], img.shape[0])


    if(variables.identification):
        if(variables.opencv):
            imgOrg= img.copy()
            imgGray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            imgGray = cv.medianBlur(imgGray, variables.blur)
            circles = cv.HoughCircles(imgGray, cv.HOUGH_GRADIENT, variables.dp, variables.minDist,
                                        param1=variables.p1, param2=variables.p2, minRadius=variables.minR, maxRadius=variables.maxR)
            if(not(circles is None)):
                circles = np.uint16(np.around(circles))
                for i in circles[0,:]:
                    printCircles(img,i[0],i[1],i[2])
    else:
        nbDetected = int(float(read_cognex("GVA026")))
        # rospy.loginfo(nbDetected)
        # Case A26 = nombre de points détecté par in-sight
        # caseCognex=29 car dans le tableur, les trous détecté commence à cette case dans notre cas
        for i in range(0,nbDetected):
            caseCognex = 29 + i*3
            if caseCognex <100:
                ptx = int(float(read_cognex("GVB0"+str(caseCognex))))
                pty = int(float(read_cognex("GVC0"+str(caseCognex))))
                ptr = int(float(read_cognex("GVD0"+str(caseCognex))))
            else:
                ptx = int(float(read_cognex("GVB"+str(caseCognex))))
                pty = int(float(read_cognex("GVC"+str(caseCognex))))
                ptr = int(float(read_cognex("GVD"+str(caseCognex))))
            printCircles(img,pty,ptx,ptr)
		

    if(save):
        cv.imwrite("%s/capture.jpg" % variables.dir, img)

    return cv.resize(img,dsize)







rospy.init_node('camera', anonymous=False)
variables.pub_ok = rospy.Publisher("camera/camera_ok", Bool, queue_size=10)
variables.pub_originale = rospy.Publisher("camera/image_originale", Image, queue_size=10)
variables.pub_annotee = rospy.Publisher("camera/image_annotee", Image, queue_size=10)
variables.pub_points = rospy.Publisher("camera/image_points", Liste_points, queue_size=10)
cognex_connect()
s1 = rospy.Service("camera/capture", capture, detection)
s2 = rospy.Service("camera/identification", identification, identify)

if variables.render:
	cv.namedWindow('Identification')
	cv.resizeWindow('Identification', 800, 200)
	cv.createTrackbar('opencv', 'Identification', variables.opencv, 1, changeMethodEvent)
	cv.createTrackbar('gain', 'Identification', variables.gain, 200, gainEvent)
	cv.createTrackbar('blur', 'Identification', variables.blur, 50, blurEvent)
	cv.createTrackbar('dp', 'Identification', variables.dp, 50, dpEvent)
	cv.createTrackbar('minDist', 'Identification', variables.minDist, 500, minDistEvent)
	cv.createTrackbar('p1', 'Identification', variables.p1, 500, p1Event)
	cv.createTrackbar('p2', 'Identification', variables.p2, 500, p2Event)
	cv.createTrackbar('min R', 'Identification', variables.minR, 400, minREvent)
	cv.createTrackbar('max R', 'Identification', variables.maxR, 400, maxREvent)
	cv.createTrackbar('identification', 'Identification', variables.identification, 1, identificationEvent)
	cv.createTrackbar('save', 'Identification', 0, 1, saveEvent)

prev_frame_time = time.time()
new_frame_time = 0
#cap = cv.VideoCapture(0)

while(not rospy.is_shutdown()):
	if variables.render :
	    img = get_image()
	    # img = cv.imread('/home/vm/Documents/image1.bmp')
	    img = reloadImg(img, save=False)

	    new_frame_time = time.time() 
	  
	    fps = 1/(new_frame_time-prev_frame_time) 
	    
	    prev_frame_time = new_frame_time 

	    fps = int(fps) 
	    fps = str(fps) 

	    # Display the resulting frame
	    if variables.plaque == "Plate":
		    cv.putText(img, re.sub(r"^Présent", "", str(read_cognex("GVE160"))), (775, 750), cv.FONT_HERSHEY_SIMPLEX, 3, (255, 100, 0), 3, cv.LINE_AA)
		    cv.putText(img, str(read_cognex("GVF160")), (500, 750), cv.FONT_HERSHEY_SIMPLEX, 3, (100, 100, 0), 3, cv.LINE_AA)
	    else:
		    cv.putText(img, str(read_cognex("GVE177")), (775, 750), cv.FONT_HERSHEY_SIMPLEX, 3, (255, 100, 0), 3, cv.LINE_AA)
		    cv.putText(img, str(read_cognex("GVF177")), (500, 750), cv.FONT_HERSHEY_SIMPLEX, 3, (100, 100, 0), 3, cv.LINE_AA)
	    cv.putText(img, fps, (7, 70), cv.FONT_HERSHEY_SIMPLEX, 3, (100, 255, 0), 3, cv.LINE_AA)
	    cv.imshow('frame',img)

	    if cv.waitKey(1) & 0xFF == ord('q'):
		    break

# When everything done, release the capture
#cap.release()
if variables.render:
	cv.destroyAllWindows()
