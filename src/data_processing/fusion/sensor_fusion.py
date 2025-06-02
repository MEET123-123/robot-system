import numpy as np
from abc import ABC, abstractmethod


class SensorFusion(ABC):
    """传感器融合抽象基类"""

    def __init__(self):
        self.sensors = []
        self.last_fused_data = None
        self.timestamp = None

    def add_sensor(self, sensor_id, weight=1.0):
        """
        添加传感器

        参数:
            sensor_id: 传感器ID
            weight: 传感器权重(默认1.0)
        """
        self.sensors.append({
            'id': sensor_id,
            'weight': weight,
            'data': None,
            'timestamp': None
        })

    def update_sensor_data(self, sensor_id, data, timestamp=None):
        """
        更新传感器数据

        参数:
            sensor_id: 传感器ID
            data: 传感器数据
            timestamp: 时间戳(可选)
        """
        for sensor in self.sensors:
            if sensor['id'] == sensor_id:
                sensor['data'] = data
                sensor['timestamp'] = timestamp or self._get_current_time()
                return True

        self.add_sensor(sensor_id)
        self.update_sensor_data(sensor_id, data, timestamp)
        return True

    @abstractmethod
    def fuse(self):
        """
        执行数据融合

        返回:
            融合后的数据
        """
        pass

    def get_fused_data(self):
        """获取最近一次融合的数据"""
        return self.last_fused_data

    def _get_current_time(self):
        """获取当前时间(用于时间戳)"""
        import time
        return time.time()