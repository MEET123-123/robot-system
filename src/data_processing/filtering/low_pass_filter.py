import numpy as np


class LowPassFilter:
    """低通滤波器实现"""

    def __init__(self, cutoff_freq, sample_freq):
        """
        初始化低通滤波器

        参数:
            cutoff_freq: 截止频率(Hz)
            sample_freq: 采样频率(Hz)
        """
        self.cutoff_freq = cutoff_freq
        self.sample_freq = sample_freq
        self.alpha = 1.0

        # 计算滤波系数
        self._compute_alpha()

        # 初始化状态
        self.last_output = None

    def _compute_alpha(self):
        """计算滤波系数"""
        dt = 1.0 / self.sample_freq
        tau = 1.0 / (2.0 * np.pi * self.cutoff_freq)
        self.alpha = dt / (dt + tau)

    def update(self, data):
        """
        更新滤波器并返回滤波后的数据

        参数:
            data: 新的输入数据

        返回:
            滤波后的数据
        """
        # 如果是第一次调用，直接返回输入数据
        if self.last_output is None:
            self.last_output = data
            return data

        # 应用低通滤波
        filtered = self.alpha * data + (1 - self.alpha) * self.last_output
        self.last_output = filtered

        return filtered

    def reset(self):
        """重置滤波器"""
        self.last_output = None


class MultiChannelLowPassFilter:
    """多通道低通滤波器实现"""

    def __init__(self, cutoff_freq, sample_freq, num_channels):
        """
        初始化多通道低通滤波器

        参数:
            cutoff_freq: 截止频率(Hz)
            sample_freq: 采样频率(Hz)
            num_channels: 通道数量
        """
        self.num_channels = num_channels
        self.filters = [
            LowPassFilter(cutoff_freq, sample_freq)
            for _ in range(num_channels)
        ]

    def update(self, data):
        """
        更新所有通道的滤波器并返回滤波后的数据

        参数:
            data: 新的输入数据(数组或列表)

        返回:
            滤波后的数据(数组)
        """
        # 确保数据长度匹配通道数量
        if len(data) != self.num_channels:
            raise ValueError(f"输入数据长度({len(data)})与通道数量({self.num_channels})不匹配")

        # 对每个通道应用滤波
        return np.array([
            self.filters[i].update(data[i])
            for i in range(self.num_channels)
        ])

    def reset(self):
        """重置所有通道的滤波器"""
        for filter in self.filters:
            filter.reset()