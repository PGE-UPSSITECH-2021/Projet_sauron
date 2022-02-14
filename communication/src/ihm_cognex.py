#!/usr/bin/env python
# coding: utf-8

from Tkinter import *
from PIL import Image
import rospy
import telnetlib
from ftplib import FTP
import ftplib
import time
import cv2 as cv
import numpy as np
from variables_cognex import Variables
from std_msgs.msg import Bool
from communication.msg import Liste_points, Points
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from communication.srv import capture
from communication.srv import identification, identificationResponse

global variables
variables = Variables()


##############################################
###########       CONNEXION       ############
##############################################

# Check of the connexion
# In case of failure, sends to camera/camera_ok False
def check_connexion(func):
    def wrap(*args, **kwargs):
        try:
            result = func(*args, **kwargs)
            variables.pub_ok.publish(True)
            return result
        except ftplib.all_errors:
            variables.pub_ok.publish(False)
            cognex_connect()

    return wrap


# Connect to the camera
@check_connexion
def cognex_connect():
    # Creation of the Telnet connexion
    variables.tn = telnetlib.Telnet(variables.ip, port=23, timeout=10)
    # We enter the username
    variables.tn.read_until("User: ", timeout=3)
    variables.tn.write(variables.user+'\r\n')
    # We enter the associated password
    variables.tn.read_until("Password: ", timeout=3)
    variables.tn.write("\r\n") 
    variables.tn.read_until("User Logged In\r\n", timeout=3)
    
    variables.tn.write("Put Live 1\r\n")
    rospy.loginfo("Com OK: "+str(variables.tn.expect(["0\r\n", "1\r\n"], timeout=3)[2]))
    # We put online the system to be able to trigger the acquisition
    variables.tn.write("SO1\r\n")
    rospy.loginfo("Com OK: "+str(variables.tn.expect(["0\r\n", "1\r\n"], timeout=3)[2]))

    # Creation of the FTP connexion
    variables.ftp = FTP(variables.ip)
    variables.ftp.login(variables.user)

    # Update of default gain value
    variables.gain = int(float(read_cognex("GVG003")))


# Read a value from the camera
@check_connexion
def read_cognex(case):
    # Send the command with Telnet
    variables.tn.write(case+"\r\n")
    # Wait for the return code
    errorCode = str(variables.tn.read_until("1\r\n", timeout=3))
    # Read of the returned value if the command did not fail
    val = variables.tn.expect(["\r\n"], timeout=3)[2]
    val = val[:len(val)-2]
    # Log info to see what is happening
    rospy.loginfo("Com : "+case+" -- "+"Com OK: "+errorCode[0]+" -- "+"Com Val : "+val+"\r\n")
    return val


# Write a value to the camera
@check_connexion
def write_cognex(case, val):
    # Send the command and associated value with Telnet
    variables.tn.write(case+str(val)+"\r\n")
    # Log info to see what is happening
    rospy.loginfo("Com : "+case+" -- "+"Com OK: "+str(variables.tn.expect(["0\r\n", "1\r\n"], timeout=3)[2]))


# Get image from the camera
@check_connexion
def get_image():
	# Send an acquisition event and wait for it
	variables.tn.write("SW8\r\n")
    # Log info to see what is happening
	rospy.loginfo("Com : SW8 -- Com OK: "+str(variables.tn.expect(["0\r\n", "1\r\n"], timeout=6)[2]))

    # Creation of the FTP connexion
    	variables.ftp = FTP(variables.ip)
    	variables.ftp.login(variables.user)

    # Download file from cognex
	filename = 'image.bmp'
	lf = open(filename, "wb")
	variables.ftp.retrbinary("RETR " + filename, lf.write)
	lf.close()
	img = cv.imread(filename)

	return img





##############################################
##########       ROS EXCHANGES       #########
##############################################
def detection(msg):
    # Get image from the camera
    img = get_image()
    # Try using the OpenCV bridge to convert image to the right format
    try:
        bridge = CvBridge()
        variables.originale = bridge.cv2_to_imgmsg(img, "bgr8")
    except CvBridgeError as e:
        print(e)

    # Publish to the associated node(s) and also return value(s)
    return variables.originale


def identify(msg):
    # Get image from the camera
    img = get_image()
    # Try using the OpenCV bridge to convert image to the right format
    try:
        bridge = CvBridge()
        if(msg.plaque == "Courbee"):
	    img = img[:, (img.shape[1]/2) - 150:(img.shape[1]/2) + 150]
            variables.originale = bridge.cv2_to_imgmsg(img, "bgr8")
        else:
            variables.originale = bridge.cv2_to_imgmsg(img, "bgr8")
    except CvBridgeError as e:
        print(e)

    if(variables.opencv):
        # Change to grayscale
        imgGray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        # Apply a median blur
        imgGray = cv.medianBlur(imgGray, variables.blur)

        # Find circles in the image
        circles = cv.HoughCircles(imgGray, cv.HOUGH_GRADIENT, variables.dp, variables.minDist,
                                    param1=variables.p1, param2=variables.p2, minRadius=variables.minR, maxRadius=variables.maxR)

        # Create list with the circles found and associate to each of them their type
        variables.points = []
        if(circles is not None):
            circles = np.uint16(np.around(circles))
            for i in circles[0,:]:
                variables.points.append(Points())
                variables.points[-1].x = i[0]
                variables.points[-1].y = i[1]
                variables.points[-1].type = variables.get_type(i[2])
                printCircles(img, i[0], i[1], i[2])

    # If the selected method is In-Sight
    else:
        # Get the number of detected circles in image
        nbDetected = int(float(read_cognex("GVA026")))
        # Create list with the circles found and associate to each of them their type
        variables.points = []
        # Get the x,y pixel position with the associed radius
        for i in range(0,nbDetected):
            caseCognex = 29 + i*3
            try:
                if caseCognex < 100:
                    ptx = int(float(read_cognex("GVB0"+str(caseCognex))))
                    pty = int(float(read_cognex("GVC0"+str(caseCognex))))
                    ptr = int(float(read_cognex("GVD0"+str(caseCognex))))
                else:
                    ptx = int(float(read_cognex("GVB"+str(caseCognex))))
                    pty = int(float(read_cognex("GVC"+str(caseCognex))))
                    ptr = int(float(read_cognex("GVD"+str(caseCognex))))
                variables.points.append(Points())
                variables.points[-1].x = ptx
                variables.points[-1].y = pty
                variables.points[-1].type = variables.get_type(ptr)
                printCircles(img,pty,ptx,ptr)

            except (ValueError, TypeError):
                rospy.loginfo("#ERR when reading points locations")

    # Try using the OpenCV bridge to convert image to the right format
    try:
        bridge = CvBridge()
        if(variables.plaque == "Courbee"):
            variables.annotee = bridge.cv2_to_imgmsg(img, "bgr8")
        else:
            variables.annotee = bridge.cv2_to_imgmsg(img, "bgr8")
    except CvBridgeError as e:
        print(e)

    print(variables.annotee.height)
    print(variables.annotee.width)

    res = identificationResponse()
    res.points.points = variables.points
    res.originale = variables.originale
    res.annotee = variables.annotee

    return res






##############################################
##########       GUI INTERFACE       #########
##############################################

#### Parameters for the circle identification ####
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
#### Parameters for the circle identification ####


# Gain applied to the image
def gainEvent(val):
    variables.gain = val
    write_cognex("SIG003", variables.gain)

# Selection of the method to apply
def changeMethodEvent(val):
    variables.opencv = False if val==0 else True

# Weither to simply acquire an image or to also apply identification 
def identificationEvent(val):
    variables.identification = False if val==0 else True

# Save of the current acquired image
def saveEvent(val):
    img = get_image()
    reloadImg(img, save=True)

# Display the circles found in the image
def printCircles(img, ptx, pty, rayon):
    # Use of a different color for each type of circles
    cv.circle(img, (ptx,pty), rayon, variables.get_color(rayon), 4)
    cv.circle(img, (ptx,pty), 2, (0,0,255), 6)

def reloadImg(img, save=False):
    # Resize image so it can be display on screen
    width = int(img.shape[1] * variables.scale_percent)
    height = int(img.shape[0] * variables.scale_percent)
    dsize = (width, height)

    # If identification needs to be performed
    if(variables.identification):
        # If the selected method is OpenCV
        if(variables.opencv):
            # Change to grayscale
            imgGray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            # Apply a median blur
            imgGray = cv.medianBlur(imgGray, variables.blur)

            # Find circles in the image
            circles = cv.HoughCircles(imgGray, cv.HOUGH_GRADIENT, variables.dp, variables.minDist,
                                        param1=variables.p1, param2=variables.p2, minRadius=variables.minR, maxRadius=variables.maxR)

            if(circles is not None):
                circles = np.uint16(np.around(circles))
                # Display the circles found in the image
                for i in circles[0,:]:
                    printCircles(img, i[0], i[1], i[2])

        # If the selected method is In-Sight
        else:
            # Get the number of detected circles in image
            nbDetected = int(float(read_cognex("GVA026")))
            # Get the x,y pixel position with the associed radius
            for i in range(0,nbDetected):
                caseCognex = 29 + i*3
                try:
                    if caseCognex < 100:
                        ptx = int(float(read_cognex("GVB0"+str(caseCognex))))
                        pty = int(float(read_cognex("GVC0"+str(caseCognex))))
                        ptr = int(float(read_cognex("GVD0"+str(caseCognex))))
                    else:
                        ptx = int(float(read_cognex("GVB"+str(caseCognex))))
                        pty = int(float(read_cognex("GVC"+str(caseCognex))))
                        ptr = int(float(read_cognex("GVD"+str(caseCognex))))
                    # Display the circles found in the image
                    printCircles(img,pty,ptx,ptr)
                except (ValueError, TypeError):
                    print("#ERR")
			
		
    # If the image has to be saved
    if(save):
        cv.imwrite("%s/capture.jpg" % variables.dir, img)

    return cv.resize(img, dsize)






# Initialize the node
rospy.init_node('camera', anonymous=False)
# Initialize the topics
variables.pub_ok = rospy.Publisher("camera/camera_ok", Bool, queue_size=10)
# Connect to the camera
cognex_connect()
# Initialize the services
s1 = rospy.Service("camera/capture", capture, detection)
s2 = rospy.Service("camera/identification", identification, identify)

# Create panel with sliders if render in True
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

# Initialize frame time
prev_frame_time = time.time()
new_frame_time = 0

while(not rospy.is_shutdown()):
    if variables.render :
        # Get image from the camera
        img = get_image()
        img = reloadImg(img, save=False)

        # Update frame time, compute and display fps
        new_frame_time = time.time() 
        fps = 1/(new_frame_time-prev_frame_time) 
        fps = str(int(fps))
        prev_frame_time = new_frame_time
        cv.putText(img, fps, (7, 70), cv.FONT_HERSHEY_SIMPLEX, 3, (100, 255, 0), 3, cv.LINE_AA)

        # Read orientation of the if available
        if variables.plaque == "Plate":
            cv.putText(img, read_cognex("GVE160"), (765, 750), cv.FONT_HERSHEY_SIMPLEX, 3, (255, 100, 0), 3, cv.LINE_AA)
        elif variables.plaque == "Courbee":
            cv.putText(img, read_cognex("GVE178"), (765, 750), cv.FONT_HERSHEY_SIMPLEX, 3, (255, 100, 0), 3, cv.LINE_AA)
        elif variables.plaque == "Lourde":
            cv.putText(img, read_cognex("GVE196"), (765, 750), cv.FONT_HERSHEY_SIMPLEX, 3, (255, 100, 0), 3, cv.LINE_AA)


        # Display the resulting frame
        cv.imshow('frame',img)

        # Close windows if "q" is pressed
        if cv.waitKey(1) & 0xFF == ord('q'):
            break


# We put the system offline
variables.tn.write("SO0\r\n")
# Log info to see what is happening
rospy.loginfo("Com OK: "+str(variables.tn.expect(["0\r\n", "1\r\n"], timeout=3)[2]))
# Create panel with sliders if render in True
if variables.render:
	cv.destroyAllWindows()
