''' Helper class and functions for loading KITTI objects

Author: Ari Jin
Date: Jan. 2022
Reference: https://www.guyuehome.com/19226
'''
import numpy as np
import math


def quart_to_rpy(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - x * z))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
    return [roll, pitch, yaw]


def euler_to_quaternion(yaw, pitch, roll):
    ''' avoid 1/2=0 in python2 '''
    yaw, pitch, roll = float(yaw), float(pitch), float(roll)
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - \
        np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + \
        np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - \
        np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + \
        np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]


def euler_to_Rmatrix(yaw, pitch, roll):
    '''transform point from rotated coordinate into fixed world coordinate'''
    return np.dot(rotz(yaw), np.dot(roty(pitch), rotx(roll)))


def Rmatrix_to_euler(R):
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0
    return [x, y, z]


def rotx(t):
    ''' 3D Rotation about the x-axis. '''
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[1,  0,  0],
                     [0,  c, -s],
                     [0,  s,  c]])


def roty(t):
    ''' Rotation about the y-axis. '''
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[c,  0,  s],
                     [0,  1,  0],
                     [-s, 0,  c]])


def rotz(t):
    ''' Rotation about the z-axis. '''
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[c, -s,  0],
                     [s,  c,  0],
                     [0,  0,  1]])


def Tmatrix(roll, pitch, yaw, x, y, z):
    R = euler_to_Rmatrix(yaw, pitch, roll)
    t = np.array([x, y, z])
    T = transform_from_rot_trans(R, t)
    return T


def transform_from_rot_trans(R, t):
    ''' Transforation matrix from rotation matrix and translation vector. '''
    R = R.reshape(3, 3)
    t = t.reshape(3, 1)
    return np.vstack((np.hstack([R, t]), [0, 0, 0, 1]))  # 4*4


def inverse_rigid_rot(R):
    ''' Inverse a rigid body rotated matrix
    '''
    inv_R = np.transpose(R)
    return inv_R


def inverse_rigid_trans(Tr):
    ''' Inverse a rigid body transform matrix (3x4 as [R|t])
        [R'|-R't; 0|1]
    '''
    inv_Tr = np.zeros_like(Tr)  # 3x4 or 4x4
    inv_Tr[0:3, 0:3] = np.transpose(Tr[0:3, 0:3])
    inv_Tr[0:3, 3] = np.dot(-np.transpose(Tr[0:3, 0:3]), Tr[0:3, 3])
    if inv_Tr.shape[0] == 4:
        inv_Tr[3, 3] = 1
    return inv_Tr


def cart2hom(pts_3d):
    ''' Input: nx3 points in Cartesian
        Oupput: nx4 points in Homogeneous by pending 1
    '''
    n = pts_3d.shape[0]
    pts_3d_hom = np.hstack((pts_3d, np.ones((n, 1))))
    return pts_3d_hom


def Tcart2hom(Tcart):
    One = np.array([0, 0, 0, 1]).reshape([1, 4])
    Thom = np.vstack((Tcart, One))
    return Thom

# import tf

# def euler2q(roll, pitch, yaw):
#     quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
#     return [quaternion[0], quaternion[1], quaternion[2], quaternion[3]]


# # math
# print(euler_to_quaternion(1, 0.1, -1.0))
# # ros tf
# print(euler2q(-1.0, 0.1, 1))

#
def euclid_distance(ax, ay, bx, by):
    dis = math.sqrt(math.pow(bx - ax, 2) + math.pow(by - ay, 2))
    return dis
