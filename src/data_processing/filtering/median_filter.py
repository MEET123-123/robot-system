import numpy as np


class MedianFilter:
    """中值滤波器实现"""

    def __init__(self, window_size, dim=1):
        """
        初始化中值滤波器

        参数:
            window_size: 窗口大小(奇数)
            dim: 数据维度(默认为1)
        """
        if window_size % 2 == 0:
            raise ValueError("窗口大小必须为奇数")

        self.window_size = window_size
        self.dim = dim
        self.buffer = []

    def update(self, data):
        """
        更新滤波器并返回滤波后的数据

        参数:
            data: 新的输入数据

        返回:
            滤波后的数据
        """
        # 将数据转换为numpy数组
        data = np.array(data)

        # 初始化缓冲区
        if not self.buffer:
            self.buffer = [data] * self.window_size

        # 添加新数据到缓冲区
        self.buffer.pop(0)
        self.buffer.append(data)

        # 计算中值
        if self.dim == 1:
            return np.median(self.buffer)
        else:
            return np.median(self.buffer, axis=0)

    def reset(self):
        """重置滤波器"""
        self.buffer = []


class SlidingWindowMedianFilter:
    """滑动窗口中值滤波器实现"""

    def __init__(self, window_size, dim=1):
        """
        初始化滑动窗口中值滤波器

        参数:
            window_size: 窗口大小(奇数)
            dim: 数据维度(默认为1)
        """
        if window_size % 2 == 0:
            raise ValueError("窗口大小必须为奇数")

        self.window_size = window_size
        self.dim = dim
        self.buffer = []

    def update(self, data):
        """
        更新滤波器并返回滤波后的数据

        参数:
            data: 新的输入数据

        返回:
            滤波后的数据
        """
        # 将数据转换为numpy数组
        data = np.array(data)

        # 添加新数据到缓冲区
        self.buffer.append(data)

        # 如果缓冲区超过窗口大小，则移除最旧的数据
        if len(self.buffer) > self.window_size:
            self.buffer.pop(0)

        # 计算中值
        if self.dim == 1:
            return np.median(self.buffer)
        else:
            return np.median(self.buffer, axis=0)

    def reset(self):
        """重置滤波器"""
        self.buffer = []