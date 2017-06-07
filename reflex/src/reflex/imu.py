#!/usr/bin/env python
import rospy
import numpy as np
import math
import argparse
from reflex_msgs.msg import Hand
from reflex_msgs.srv import DistalRotation
import reflex_msgs

# quaternion_to_matrix: converts an array of 4 floats representing a quaternion to a rotation matrix
# float[4] q: the quaternion to convert
# return float[3][3]: the equivalent rotation matrix
def quaternion_to_matrix(q):
    magnitude = math.sqrt(pow(q[0], 2)+pow(q[1], 2)+pow(q[2], 2)+pow(q[3], 2))
    try:
        q = [q[i] / magnitude for i in range(4)]
        return np.array([
            [1 - 2*q[2]*q[2] - 2*q[3]*q[3], 2*q[1]*q[2] - 2*q[3]*q[0],     2*q[1]*q[3] + 2*q[2]*q[0]],
            [2*q[1]*q[2] + 2*q[3]*q[0],     1 - 2*q[1]*q[1] - 2*q[3]*q[3], 2*q[2]*q[3] - 2*q[1]*q[0]],
            [2*q[1]*q[3] - 2*q[2]*q[0],     2*q[2]*q[3] + 2*q[1]*q[0],     1 - 2*q[1]*q[1] - 2*q[2]*q[2]]])
    except ZeroDivisionError:
        print "An IMU is not providing data. Check connections."
        return np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])
    

# generate_ht: generates the homogeneous transform matrix from a rotation matrix and a translation vector
# float[3][3] rotation_matrix: the rotation matrix
# float[3][1] translation_vector: the translation vector
# return float[4][4]: the homogeneous transform matrix
def generate_ht(rotation_matrix, translation_vector):
    return np.vstack((np.hstack((rotation_matrix, translation_vector)), [0, 0, 0, 1]))

# axis_angle_to_matrix: generates the rotation matrix corresponding to the provided rotation
# float angle: the angle, in radians, through which the rotation occurs
# float[3] axis: the x, y, and z components of the axis around which the rotation takes place
# return float[3][3]: the rotation matrix describing the rotation
def axis_angle_to_matrix(angle, axis):
    c = math.cos(angle)
    s = math.sin(angle)
    t = 1 - c
    magnitude = math.sqrt(pow(axis[0], 2)+pow(axis[1], 2)+pow(axis[2], 2))
    x = axis[0] / magnitude
    y = axis[1] / magnitude
    z = axis[2] / magnitude
    return np.array([
        [t*x*x + c,   t*x*y - z*s, t*x*z + y*s],
        [t*x*y + z*s, t*y*y + c,   t*y*z - x*s],
        [t*x*z - y*s, t*y*z + x*s, t*z*z + c]])

######## NOT IMPLEMENTED YET #########
# # matrix_to_angle_axis: generates the axis and angle of rotation from a rotation matrix
# # float[3][3] m: the rotation matrix
# # return float[4]: the angle of rotation followed by the x, y, and z components of the axis of rotation
# def matrix_to_axis_angle(m):
#     angle = math.acos((np.trace(m) - 1)/2)
#     x = (m21 - m12)/math.sqrt((m21 - m12)2+(m02 - m20)2+(m10 - m01)2)
#     y = (m02 - m20)/math.sqrt((m21 - m12)2+(m02 - m20)2+(m10 - m01)2)
#     z = (m10 - m01)/math.sqrt((m21 - m12)2+(m02 - m20)2+(m10 - m01)2)

# matrix_to_euler_angle: converts a rotation matrix to its equivalent set of euler angles
# float[3][3] m: the rotation matrix to convert
# return float[3]: [roll, pitch, yaw]
def matrix_to_euler_angle(m):
    theta = -math.asin(m[2][0])
    psi = math.atan2(m[2][1]/math.cos(theta), m[2][2]/math.cos(theta))
    phi = math.atan2(m[1][0]/math.cos(theta), m[0][0]/math.cos(theta))
    return [-psi, theta, -phi]

# get_distal_rotation: determines the rotation of the distal link in 3 spatial dimensions
# float[4] palmQuat: the quaternion values from the palm IMU
# float preshape: the number of radians the preshape has rotated
# float proximal: the number of radians the proximal link rotated, as determined by the encoder
# float fingerQuat: the quaternion values from the finger IMU
# return float[3][3]: the ro
def get_distal_rotation(palmQuat, preshape, proximal, fingerQuat):
    r_palmImu = quaternion_to_matrix(palmQuat)
    r_preshape = axis_angle_to_matrix(preshape, [0, 0, 1])
    # r_preshape = axis_angle_to_matrix(0, [0, 0, 1])
    r_proximal = axis_angle_to_matrix(proximal, [-1, 0, 0])
    # r_proximal = axis_angle_to_matrix(0, [0, -1, 0])
    r_fingertipImu = quaternion_to_matrix(fingerQuat)

    r_distal = np.dot(np.dot(np.dot(np.linalg.inv(r_proximal), 
                                    np.linalg.inv(r_preshape)), 
                                    np.linalg.inv(r_palmImu)), 
                                    r_fingertipImu)

    return matrix_to_euler_angle(r_distal)
    # return r_distal

######### NOT IMPLEMENTED YET #######
# # get_fingertipImu_orientation: determines the homogeneous transform of the fingertipIMU from the palmIMU
# # float[4] palmQuat: the quaternion values from the palm IMU
# # float preshape: the number of radians the preshape has rotated
# # float proximal: the number of radians the proximal link rotated, as determined by the encoder
# # float fingerQuat: the quaternion values from the finger IMU
# def get_fingertipImu_orientation(palmQuat, preshape, proximal, fingerQuat):
#     r_palmImu = quaternion_to_matrix(palmQuat)
#     r_preshape = axis_angle_to_matrix(preshape, [0, 0, -1])
#     r_proximal = axis_angle_to_matrix(proximal, [0, -1, 0])
#     r_fingertipImu = quaternion_to_matrix(fingerQuat)

#     r_distal = np.dot(np.dot(np.dot(np.linalg.inv(r_proximal), 
#                                     np.linalg.inv(r_preshape)), 
#                                     np.linalg.inv(r_palmImu)), 
#                                     r_fingertipImu)


#     t_palmImu = np.zeros((3,1))
#     t_preshape = np.zeros((3,1))
#     t_proximal = np.zeros((3,1))
#     t_distal = np.zeros((3,1))


#     g_palmImu = generate_ht(r_palmImu, t_palmImu)
#     g_preshape = generate_ht(r_preshape, t_preshape)
#     g_proximal = generate_ht(r_proximal, t_proximal)
#     g_distal = generate_ht(r_distal, np.dot(t_distal/2,np.identity(3)+r_distal))

#     g_fingertipImu = np.dot(np.dot(np.dot(g_palmImu, 
#                                         g_preshape), 
#                                         g_proximal), 
#                                         g_distal)

#     return g_fingertipImu

def handle_distal_rotation_srv(data):
    distal_rot = get_distal_rotation(data.palm_imu_quat,
                                    data.joint_angle,
                                    data.proximal,
                                    data.finger_imu_quat)
    # rospy.loginfo("rotation: %s", str(distal_rot))
    return reflex_msgs.srv.DistalRotationResponse(distal_rot)

def distal_rotation_server():
    rospy.init_node('imu', anonymous=True)
    s = rospy.Service('distal_rotation', DistalRotation, handle_distal_rotation_srv)
    rospy.spin()

if __name__ == '__main__':
    distal_rotation_server()
