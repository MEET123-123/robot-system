import numpy as np
from .sensor_fusion import SensorFusion


class WeightedAverageFusion(SensorFusion):
    """加权平均传感器融合实现"""

    def __init__(self):
        super().__init__()

    def fuse(self):
        """
        执行加权平均融合

        返回:
            融合后的数据
        """
        valid_sensors = [s for s in self.sensors if s['data'] is not None]

        if not valid_sensors:
            return None

        # 检查所有传感器数据维度是否一致
        first_dim = len(valid_sensors[0]['data'])
        if any(len(s['data']) != first_dim for s in valid_sensors):
            raise ValueError("所有传感器数据维度必须一致")

        # 计算加权平均
        fused_data = np.zeros(first_dim)
        total_weight = 0.0

        for sensor in valid_sensors:
            weight = sensor['weight']
            fused_data += np.array(sensor['data']) * weight
            total_weight += weight

        if total_weight > 0:
            fused_data /= total_weight

        self.last_fused_data = fused_data
        self.timestamp = self._get_current_time()

        return fused_data