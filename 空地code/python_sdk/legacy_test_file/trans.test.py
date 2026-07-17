import math
import time
from dataclasses import dataclass

import numpy as np
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation


def quaternions_to_euler(w, x, y, z):
    # mathod 1
    # r = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    # p = math.asin(2 * (w * y - z * x))
    # y = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    # mathod 2
    # r = math.atan2(2.0 * (w * x + y * z), w * w - x * x - y * y + z * z)
    # p = -math.asin(2.0 * (x * z - w * y))
    # y = math.atan2(2.0 * (w * z + x * y), w * w + x * x - y * y - z * z)

    # mathod 3
    # Resolve the gimbal lock problem
    sinp = 2.0 * (w * y - z * x)
    p = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    r = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    y = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    # convert radians to degrees
    r, p, y = math.degrees(r), math.degrees(p), math.degrees(y)
    return r, p, y


def quaternions_to_rotation_matrix(w, x, y, z) -> np.ndarray:
    """
    将wxyz的四元数转换为3x3的旋转矩阵
    """
    # 构造旋转矩阵
    rotation_matrix = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )

    return rotation_matrix


def rotation_matrix_to_quaternions(rotation_matrix: np.ndarray) -> tuple:
    """
    将3x3的旋转矩阵转换为wxyz的四元数
    """
    # 计算四元数的w分量
    w = np.sqrt(1 + rotation_matrix[0, 0] + rotation_matrix[1, 1] + rotation_matrix[2, 2]) / 2
    # 计算四元数的x, y, z分量
    x = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / (4 * w)
    y = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / (4 * w)
    z = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / (4 * w)

    return w, x, y, z


def rotation_matrix_to_euler(rotation_matrix):
    r = Rotation.from_matrix(rotation_matrix)
    euler_angles = r.as_euler("xyz", degrees=True)
    return euler_angles[0], euler_angles[1], euler_angles[2]


@dataclass
class T265_Pose_Frame(object):
    """
    T265 姿态数据帧
    """

    @dataclass
    class _XYZ:  # 三维坐标
        x: float
        y: float
        z: float

    @dataclass
    class _WXYZ:  # 四元数
        w: float
        x: float
        y: float
        z: float

    translation: _XYZ  # 位移 / m
    rotation: _WXYZ  # 四元数姿态


class test:
    def __init__(self) -> None:
        self.pose = T265_Pose_Frame(T265_Pose_Frame._XYZ(7, 8, 9), T265_Pose_Frame._WXYZ(0.9, 0.1, 0.2, 0.3))

    def establish_secondary_origin(self):
        # 获取当前位置和朝向
        position = np.array([self.pose.translation.x, self.pose.translation.y, self.pose.translation.z])
        orientation = np.array([self.pose.rotation.x, self.pose.rotation.y, self.pose.rotation.z, self.pose.rotation.w])

        # 将当前位置和朝向作为副坐标系的原点和朝向
        # rotation_matrix = quaternions_to_rotation_matrix(*orientation)
        self._secondary_position = position
        self._secondary_orientation = orientation
        self._secondary_rotation = Rotation.from_quat(orientation)  # xyzw
        self._secondary_rotation_matrix_inv = self._secondary_rotation.inv().as_matrix()
        self._secondary_rotation_inv = self._secondary_rotation.inv()

    def get_pose_in_secondary_frame(self):
        # 获取当前位置和朝向
        position = np.array([self.pose.translation.x, self.pose.translation.y, self.pose.translation.z])
        orientation = np.array([self.pose.rotation.x, self.pose.rotation.y, self.pose.rotation.z, self.pose.rotation.w])

        # 将当前位置和朝向转换到副坐标系中
        position -= self._secondary_position
        # 反向应用副坐标系的旋转矩阵
        position = np.dot(position, self._secondary_rotation_matrix_inv)
        # 反向应用副坐标系的朝向
        # rotation_matrix = quaternions_to_rotation_matrix(*orientation)
        # rotation_matrix = np.dot(rotation_matrix, self._secondary_rotation_matrix.T)
        # orientation = rotation_matrix_to_quaternions(rotation_matrix)
        rotation = Rotation.from_quat(orientation) * self._secondary_rotation_inv
        orientation = rotation.as_quat()
        euler_angles = rotation.as_euler("zxy", degrees=True)

        return position, orientation, euler_angles


t = test()
t.establish_secondary_origin()
t.pose.translation.x += 1
t0 = time.perf_counter()
out = t.get_pose_in_secondary_frame()
t1 = time.perf_counter()
print(f"{out}, cost: {t1 - t0}")

import timeit

print(f'{timeit.timeit("t.get_pose_in_secondary_frame()", setup="from __main__ import t", number=10000)/10000:.9f}')
