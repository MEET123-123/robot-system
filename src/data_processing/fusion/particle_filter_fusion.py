import numpy as np
from .sensor_fusion import SensorFusion


class ParticleFilterFusion(SensorFusion):
    """基于粒子滤波的传感器融合实现"""

    def __init__(self, state_dim, num_particles=100):
        """
        初始化粒子滤波融合器

        参数:
            state_dim: 状态向量维度
            num_particles: 粒子数量(默认100)
        """
        super().__init__()

        self.state_dim = state_dim
        self.num_particles = num_particles

        # 初始化粒子和权重
        self.particles = np.random.randn(num_particles, state_dim)
        self.weights = np.ones(num_particles) / num_particles

        # 粒子滤波参数
        self.process_noise = np.eye(state_dim) * 0.1
        self.measurement_noise = np.eye(state_dim) * 0.1
        self.resample_threshold = num_particles / 2

    def set_process_noise(self, noise):
        """设置过程噪声协方差"""
        self.process_noise = np.array(noise)

    def set_measurement_noise(self, noise):
        """设置测量噪声协方差"""
        self.measurement_noise = np.array(noise)

    def _system_model(self, x, dt):
        """系统模型(状态转移函数)"""
        # 简化的线性模型，实际应用中应替换为真实系统模型
        return x

    def _measurement_model(self, x, sensor_id=None):
        """测量模型"""
        # 简化的线性模型，实际应用中应替换为真实测量模型
        return x

    def _compute_likelihood(self, z, x, sensor_id=None):
        """计算似然函数"""
        # 计算预测测量
        z_pred = self._measurement_model(x, sensor_id)

        # 计算误差
        error = z - z_pred

        # 计算似然(高斯分布)
        likelihood = np.exp(-0.5 * np.sum(error * error))

        return likelihood

    def _resample(self):
        """重采样步骤"""
        # 计算有效粒子数
        neff = 1.0 / np.sum(self.weights ** 2)

        # 如果有效粒子数小于阈值，则执行重采样
        if neff < self.resample_threshold:
            indices = np.random.choice(
                self.num_particles,
                self.num_particles,
                p=self.weights
            )

            self.particles = self.particles[indices]
            self.weights = np.ones(self.num_particles) / self.num_particles

    def fuse(self):
        """
        执行粒子滤波融合

        返回:
            融合后的数据
        """
        valid_sensors = [s for s in self.sensors if s['data'] is not None]

        if not valid_sensors:
            return None

        # 预测步骤: 应用系统模型并添加噪声
        for i in range(self.num_particles):
            # 应用系统模型
            self.particles[i] = self._system_model(self.particles[i], 0.1)

            # 添加过程噪声
            noise = np.random.multivariate_normal(
                np.zeros(self.state_dim),
                self.process_noise
            )
            self.particles[i] += noise

        # 更新步骤: 更新粒子权重
        for sensor in valid_sensors:
            sensor_data = np.array(sensor['data'])

            for i in range(self.num_particles):
                # 计算似然
                likelihood = self._compute_likelihood(
                    sensor_data,
                    self.particles[i],
                    sensor['id']
                )

                # 更新权重
                self.weights[i] *= likelihood

        # 归一化权重
        if np.sum(self.weights) > 0:
            self.weights /= np.sum(self.weights)
        else:
            # 如果所有权重都为零，重新初始化
            self.weights = np.ones(self.num_particles) / self.num_particles

        # 重采样
        self._resample()

        # 计算融合结果(加权平均)
        fused_data = np.sum(self.particles * self.weights[:, np.newaxis], axis=0)

        self.last_fused_data = fused_data
        self.timestamp = self._get_current_time()

        return fused_data