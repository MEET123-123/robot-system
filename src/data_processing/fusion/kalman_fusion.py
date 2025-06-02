import numpy as np
from .sensor_fusion import SensorFusion
from filtering.kalman_filter import KalmanFilter


class KalmanFusion(SensorFusion):
    """基于卡尔曼滤波的传感器融合实现"""

    def __init__(self, state_dim, measurement_dim):
        """
        初始化卡尔曼滤波融合器

        参数:
            state_dim: 状态向量维度
            measurement_dim: 测量向量维度
        """
        super().__init__()

        # 初始化卡尔曼滤波器
        self.kalman_filter = KalmanFilter(state_dim, measurement_dim)

        # 设置默认矩阵
        self.kalman_filter.set_transition_matrix(np.eye(state_dim))
        self.kalman_filter.set_measurement_matrix(np.eye(measurement_dim, state_dim))

        # 过程噪声和测量噪声协方差
        self.kalman_filter.set_process_noise(np.eye(state_dim) * 0.01)
        self.kalman_filter.set_measurement_noise(np.eye(measurement_dim) * 0.1)

        # 初始状态和误差协方差
        self.kalman_filter.set_initial_state(np.zeros((state_dim, 1)))
        self.kalman_filter.P = np.eye(state_dim) * 100.0

    def set_transition_matrix(self, F):
        """设置状态转移矩阵"""
        self.kalman_filter.set_transition_matrix(F)

    def set_measurement_matrix(self, H):
        """设置测量矩阵"""
        self.kalman_filter.set_measurement_matrix(H)

    def set_process_noise(self, Q):
        """设置过程噪声协方差"""
        self.kalman_filter.set_process_noise(Q)

    def set_measurement_noise(self, R):
        """设置测量噪声协方差"""
        self.kalman_filter.set_measurement_noise(R)

    def fuse(self):
        """
        执行卡尔曼滤波融合

        返回:
            融合后的数据
        """
        valid_sensors = [s for s in self.sensors if s['data'] is not None]

        if not valid_sensors:
            return None

        # 预测步骤
        self.kalman_filter.predict()

        # 对于每个传感器，执行更新步骤
        for sensor in valid_sensors:
            # 将传感器数据转换为测量向量
            z = np.array(sensor['data']).reshape(self.kalman_filter.dim_z, 1)

            # 根据传感器权重调整测量噪声
            # 权重越高，噪声越小
            R = self.kalman_filter.R / sensor['weight']
            self.kalman_filter.set_measurement_noise(R)

            # 执行更新
            self.kalman_filter.update(z)

        # 获取融合后的状态
        fused_data = self.kalman_filter.x.flatten()

        self.last_fused_data = fused_data
        self.timestamp = self._get_current_time()

        return fused_data