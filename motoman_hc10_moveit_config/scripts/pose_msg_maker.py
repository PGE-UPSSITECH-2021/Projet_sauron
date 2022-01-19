#!/usr/bin/env python
import geometry_msgs.msg
import numpy as np
from pyquaternion import Quaternion

def homogeneous_matrix_to_pose_msg(mat):
    # message creation
    pose = geometry_msgs.msg.Pose()

    # Extracting the translation
    pose.position.x = mat[0,3]
    pose.position.y = mat[1,3]
    pose.position.z = mat[2,3]

    # Conversion of the rotation matrix into a quaternion
    q = Quaternion(matrix=mat)
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]

    return pose