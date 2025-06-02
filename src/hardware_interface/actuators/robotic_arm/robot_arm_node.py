import rtde_control
import rtde_receive
import numpy as np
import threading
import time
from enum import Enum

from PyQt5.QtGui.QTextCursor import position
from prompt_toolkit.key_binding.bindings.named_commands import self_insert


class RobotArm:
    """手术机器人机械臂控制节点"""

    class ArmState(Enum):
        IDLE = 0
        MOVING = 1
        PAUSED = 2
        ERROR = 3

    def __init__(self,ip_adress):

        # 状态变量
        self.ip_adress = ip_adress
        self.state = self.ArmState.IDLE
        self.current_joint_positions = []
        self.current_tcp_positions = []
        self.current_pose = None
        self.joint_names = [
            'joint_1', 'joint_2', 'joint_3',
            'joint_4', 'joint_5', 'joint_6'
        ]
        # 机械臂数据接口
        self.rtde_r = rtde_receive.RTDEReceiveInterface(self.ip_adress)
        self.rtde_c = rtde_control.RTDEControlInterface(self.ip_adress)

        # 机械臂DH参数表
        self.a = np.array([0, -0.24365, -0.21300, 0, 0, 0])
        self.d = np.array([0.15190, 0, 0, 0.08340, 0.08340, 0.08240])

    def move_to_joint_positions(self, joint_positions):
        """移动机械臂到指定关节位置

        Args:
            joint_positions: 目标关节位置列表

        Returns:
            bool: 是否成功发送运动命令
        """
        # 发送目标
        self.state = self.ArmState.MOVING
        self.rtde_c.moveJ(joint_positions,0.1,0.1)
        self.current_pose = self.rtde_c.getPose()
        self.current_tcp_positions = self.rtde_r.getActualTCPPose()
        return True

    def inv_kinematics(self,T):
        # 矩阵赋值便于计算
        nx = T[0, 0]
        ny = T[1, 0]
        nz = T[2, 0]
        ox = T[0, 1]
        oy = T[1, 1]
        oz = T[2, 1]
        ax = T[0, 2]
        ay = T[1, 2]
        az = T[2, 2]
        px = T[0, 3]
        py = T[1, 3]
        pz = T[2, 3]

        # 求关节角1
        m = self.d[5] * ay - py
        n = ax * self.d[5] - px
        theta1_1 = np.arctan2(m, n) - np.arctan2(self.d[3], np.sqrt(m ** 2 + n ** 2 - self.d[3] ** 2))
        theta1_2 = np.arctan2(m, n) - np.arctan2(self.d[3], -np.sqrt(m ** 2 + n ** 2 - self.d[3] ** 2))
        theta1 = np.array([theta1_1, theta1_2])

        # 求关节角5
        theta5_1 = np.arccos(ax * np.sin(theta1) - ay * np.cos(theta1))
        theta5_2 = -np.arccos(ax * np.sin(theta1) - ay * np.cos(theta1))
        theta5 = np.vstack((theta5_1, theta5_2))

        # 求关节角6
        mm = nx * np.sin(theta1) - ny * np.cos(theta1)
        nn = ox * np.sin(theta1) - oy * np.cos(theta1)
        theta6_1 = np.arctan2(mm, nn) - np.arctan2(np.sin(theta5[0, :]), 0)
        theta6_2 = np.arctan2(mm, nn) - np.arctan2(np.sin(theta5[1, :]), 0)
        theta6 = np.vstack((theta6_1, theta6_2))

        # 求关节角3
        mmm_1 = self.d[4] * (np.sin(theta6[0, :]) * (nx * np.cos(theta1) + ny * np.sin(theta1)) + np.cos(theta6[0, :]) * (
                    ox * np.cos(theta1) + oy * np.sin(theta1))) \
                - self.d[5] * (ax * np.cos(theta1) + ay * np.sin(theta1)) + px * np.cos(theta1) + py * np.sin(theta1)
        nnn_1 = pz - self.d[0] - az * self.d[5] + self.d[4] * (oz * np.cos(theta6[0, :]) + nz * np.sin(theta6[0, :]))
        mmm_2 = self.d[4] * (np.sin(theta6[1, :]) * (nx * np.cos(theta1) + ny * np.sin(theta1)) + np.cos(theta6[1, :]) * (
                    ox * np.cos(theta1) + oy * np.sin(theta1))) \
                - self.d[5] * (ax * np.cos(theta1) + ay * np.sin(theta1)) + px * np.cos(theta1) + py * np.sin(theta1)
        nnn_2 = pz - self.d[0] - az * self.d[5] + self.d[4] * (oz * np.cos(theta6[1, :]) + nz * np.sin(theta6[1, :]))
        mmm = np.vstack((mmm_1, mmm_2))
        nnn = np.vstack((nnn_1, nnn_2))
        theta3_1 = np.arccos((mmm ** 2 + nnn ** 2 - self.a[1] ** 2 - self.a[2] ** 2) / (2 * self.a[1] * self.a[2]))
        theta3_2 = -np.arccos((mmm ** 2 + nnn ** 2 - self.a[1] ** 2 - self.a[2] ** 2) / (2 * self.a[1] * self.a[2]))
        theta3 = np.vstack((theta3_1, theta3_2))

        # 求关节角2
        mmm_s2 = np.vstack((mmm, mmm))
        nnn_s2 = np.vstack((nnn, nnn))
        s2 = ((self.a[2] * np.cos(theta3) + self.a[1]) * nnn_s2 - self.a[2] * np.sin(theta3) * mmm_s2) / \
             (self.a[1] ** 2 + self.a[2] ** 2 + 2 * self.a[1] * self.a[2] * np.cos(theta3))
        c2 = (mmm_s2 + self.a[2] * np.sin(theta3) * s2) / (self.a[2] * np.cos(theta3) + self.a[1])
        theta2 = np.arctan2(s2, c2)

        # 整理关节角1 5 6 3 2
        theta = np.zeros((8, 6))
        theta[:4, 0] = theta1[0]
        theta[4:, 0] = theta1[1]
        theta[:, 1] = [theta2[0, 0], theta2[2, 0], theta2[1, 0], theta2[3, 0], theta2[0, 1], theta2[2, 1], theta2[1, 1],
                       theta2[3, 1]]
        theta[:, 3] = [theta3[0, 0], theta3[2, 0], theta3[1, 0], theta3[3, 0], theta3[0, 1], theta3[2, 1], theta3[1, 1],
                       theta3[3, 1]]
        theta[:2, 5] = theta5[0, 0]
        theta[2:4, 5] = theta5[1, 0]
        theta[4:6, 5] = theta5[0, 1]
        theta[6:, 5] = theta5[1, 1]
        # 修正索引错误，原代码中theta[:2, 6]等超出了数组范围
        theta[:2, 4] = theta6[0, 0]
        theta[2:4, 4] = theta6[1, 0]
        theta[4:6, 4] = theta6[0, 1]
        theta[6:, 4] = theta6[1, 1]

        # 求关节角4
        theta[:, 2] = np.arctan2(
            -np.sin(theta[:, 4]) * (nx * np.cos(theta[:, 0]) + ny * np.sin(theta[:, 0])) - np.cos(theta[:, 4]) *
            (ox * np.cos(theta[:, 0]) + oy * np.sin(theta[:, 0])),
            oz * np.cos(theta[:, 4]) + nz * np.sin(theta[:, 4])) - theta[:, 1] - theta[:, 3]

        return theta

    # 选择变化最小的一组关节角度
    def select_min_change_angles(self,joint_angles, prev_angles=None):
        if prev_angles is None:
            # 如果没有上一组角度，默认选择第一组
            return joint_angles[0]

        # 计算每组关节角度与上一组角度的差值的平方和
        differences = np.sum((joint_angles - prev_angles) ** 2, axis=1)
        # 找到差值平方和最小的索引
        min_index = np.argmin(differences)
        return joint_angles[min_index]

    @staticmethod
    # 从欧拉角生成旋转矩阵
    def euler_to_rotation_matrix(rx, ry, rz):
        """
        将欧拉角（rx, ry, rz）转换为旋转矩阵
        """
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(rx), -np.sin(rx)],
            [0, np.sin(rx), np.cos(rx)]
        ])
        Ry = np.array([
            [np.cos(ry), 0, np.sin(ry)],
            [0, 1, 0],
            [-np.sin(ry), 0, np.cos(ry)]
        ])
        Rz = np.array([
            [np.cos(rz), -np.sin(rz), 0],
            [np.sin(rz), np.cos(rz), 0],
            [0, 0, 1]
        ])
        R = np.dot(Rz, np.dot(Ry, Rx))
        return R

    def move_to_pose(self, pose):
        """移动机械臂到指定位姿(在基坐标系下)

        Args:
            pose: 目标位姿 [x, y, z, rx, ry, rz] (单位: 米, 弧度)

        Returns:
            bool: 是否成功发送运动命令
        """
        position = pose[:3]
        rx, ry, rz = pose[3:]
        R = self.euler_to_rotation_matrix(rx, ry, rz)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = position
        # 计算当前位置的所有可能关节角度
        all_joint_angles = self.inv_kinematics(T)
        # 选择变化最小的一组关节角度
        selected_angles = self.select_min_change_angles(all_joint_angles, self.current_pose)

        return self.move_to_joint_positions(selected_angles)

    def get_status(self):
        """获取机械臂状态"""
        return {
            'state': self.state.name,
            'joint_positions': self.current_joint_positions,
            'current_pose': self.current_pose
        }

    def close(self):
        # 关闭接口
        self.rtde_r.disconnect()
        self.rtde_c.disconnect()


def main(args=None):
    ip_adress = "192.168.56.1"
    node = RobotArm(ip_adress)
    x1 = [-0.29286, -0.11026, 0.41553, 2.3173, 2.2869, -0.2327]
    x2 = [-0.38097, -0.10678, 0.34177, 2.3134, 2.2709, -0.2107]
    x3 = [-0.24551, -0.12168, 0.40281, 2.2787, 2.3706, -0.1925]
    try:
        node.move_to_joint_positions(x1)
        node.move_to_joint_positions(x2)
        node.move_to_joint_positions(x3)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()

if __name__ == '__main__':
    main()