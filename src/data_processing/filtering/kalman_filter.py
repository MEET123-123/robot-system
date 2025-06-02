import numpy as np


class KalmanFilter:
    """卡尔曼滤波器实现"""

    def __init__(self, dim_x, dim_z, dim_u=0):
        """
        初始化卡尔曼滤波器

        参数:
            dim_x: 状态向量维度
            dim_z: 测量向量维度
            dim_u: 控制向量维度(默认为0)
        """
        self.dim_x = dim_x
        self.dim_z = dim_z
        self.dim_u = dim_u

        # 状态向量
        self.x = np.zeros((dim_x, 1))

        # 状态转移矩阵
        self.F = np.eye(dim_x)

        # 测量矩阵
        self.H = np.zeros((dim_z, dim_x))

        # 过程噪声协方差
        self.Q = np.eye(dim_x)

        # 测量噪声协方差
        self.R = np.eye(dim_z)

        # 后验误差协方差
        self.P = np.eye(dim_x)

        # 卡尔曼增益
        self.K = np.zeros((dim_x, dim_z))

        # 单位矩阵
        self.I = np.eye(dim_x)

        # 时间间隔
        self.dt = 0.01

        # 上次更新时间
        self.last_update_time = None

    def predict(self, u=None):
        """
        预测步骤: 根据系统动态模型预测下一时刻状态

        参数:
            u: 控制输入向量

        返回:
            预测的状态向量
        """
        # 如果提供了控制输入，则应用控制模型
        if u is not None:
            self.x = self.F @ self.x + self.B @ u
        else:
            self.x = self.F @ self.x

        # 更新误差协方差
        self.P = self.F @ self.P @ self.F.T + self.Q

        return self.x.copy()

    def update(self, z):
        """
        更新步骤: 根据测量值更新预测状态

        参数:
            z: 测量向量

        返回:
            更新后的状态向量
        """
        # 计算卡尔曼增益
        S = self.H @ self.P @ self.H.T + self.R
        self.K = self.P @ self.H.T @ np.linalg.inv(S)

        # 更新状态估计
        y = z - self.H @ self.x  # 测量残差
        self.x = self.x + self.K @ y

        # 更新误差协方差
        I_KH = self.I - self.K @ self.H
        self.P = (I_KH @ self.P) @ I_KH.T + self.K @ self.R @ self.K.T

        return self.x.copy()

    def set_initial_state(self, x0):
        """设置初始状态"""
        self.x = np.array(x0).reshape(self.dim_x, 1)

    def set_transition_matrix(self, F):
        """设置状态转移矩阵"""
        self.F = np.array(F)

    def set_measurement_matrix(self, H):
        """设置测量矩阵"""
        self.H = np.array(H)

    def set_process_noise(self, Q):
        """设置过程噪声协方差"""
        self.Q = np.array(Q)

    def set_measurement_noise(self, R):
        """设置测量噪声协方差"""
        self.R = np.array(R)

    def set_control_matrix(self, B):
        """设置控制矩阵"""
        self.B = np.array(B)

    def set_time_interval(self, dt):
        """设置时间间隔"""
        self.dt = dt


class ExtendedKalmanFilter(KalmanFilter):
    """扩展卡尔曼滤波器实现"""

    def __init__(self, dim_x, dim_z, dim_u=0):
        """
        初始化扩展卡尔曼滤波器

        参数:
            dim_x: 状态向量维度
            dim_z: 测量向量维度
            dim_u: 控制向量维度(默认为0)
        """
        super().__init__(dim_x, dim_z, dim_u)

    def predict(self, u=None, f=None):
        """
        预测步骤: 使用非线性系统模型预测下一时刻状态

        参数:
            u: 控制输入向量
            f: 非线性状态转移函数

        返回:
            预测的状态向量
        """
        if f is None:
            # 如果没有提供非线性函数，则使用线性模型
            return super().predict(u)

        # 应用非线性状态转移函数
        self.x = f(self.x, u, self.dt)

        # 更新误差协方差 (使用雅可比矩阵近似)
        self.P = self.F @ self.P @ self.F.T + self.Q

        return self.x.copy()

    def update(self, z, h=None):
        """
        更新步骤: 使用非线性测量模型更新预测状态

        参数:
            z: 测量向量
            h: 非线性测量函数

        返回:
            更新后的状态向量
        """
        if h is None:
            # 如果没有提供非线性函数，则使用线性模型
            return super().update(z)

        # 计算卡尔曼增益
        S = self.H @ self.P @ self.H.T + self.R
        self.K = self.P @ self.H.T @ np.linalg.inv(S)

        # 更新状态估计
        y = z - h(self.x)  # 测量残差
        self.x = self.x + self.K @ y

        # 更新误差协方差
        I_KH = self.I - self.K @ self.H
        self.P = (I_KH @ self.P) @ I_KH.T + self.K @ self.R @ self.K.T

        return self.x.copy()