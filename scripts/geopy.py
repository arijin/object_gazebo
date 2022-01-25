import numpy as np
import math


def quart_to_rpy(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - x * z))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
    return [roll, pitch, yaw]


def euler_to_quaternion(yaw, pitch, roll):
    # avoid 1/2=0 in python2
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
