import numpy as np


class SensorCoordinateSystem:
    """多传感器坐标系统一管理类"""

    def __init__(self):
        """初始化坐标系统一类"""
        self.sensors = {}  # 存储传感器参数的字典

    def add_sensor(self, sensor_id, translation, rotation, parent_frame="world"):
        """
        添加一个新传感器及其坐标变换参数

        参数:
            sensor_id: 传感器唯一标识符
            translation: 传感器在父坐标系中的位置 [x, y, z]
            rotation: 传感器的旋转矩阵 (3x3 numpy数组) 或四元数 [qw, qx, qy, qz]
            parent_frame: 父坐标系，默认为世界坐标系
        """
        # 转换平移向量为numpy数组
        t = np.array(translation).reshape(3, 1)

        # 处理旋转表示（矩阵或四元数）
        if isinstance(rotation, np.ndarray) and rotation.shape == (3, 3):
            R = rotation
        else:
            # 假设输入是四元数 [qw, qx, qy, qz]
            q = np.array(rotation)
            R = self.quaternion_to_rotation_matrix(q)

        # 构建齐次变换矩阵
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t.flatten()

        # 存储传感器参数
        self.sensors[sensor_id] = {
            'parent_frame': parent_frame,
            'transform': T,
            'inverse_transform': np.linalg.inv(T)
        }

        print(f"已添加传感器 {sensor_id}，父坐标系: {parent_frame}")

    def quaternion_to_rotation_matrix(self, q):
        """将四元数转换为旋转矩阵"""
        qw, qx, qy, qz = q

        # 归一化四元数
        norm = np.linalg.norm(q)
        if norm != 0:
            qw, qx, qy, qz = q / norm

        # 计算旋转矩阵
        R = np.array([
            [1 - 2 * qy * qy - 2 * qz * qz, 2 * qx * qy - 2 * qz * qw, 2 * qx * qz + 2 * qy * qw],
            [2 * qx * qy + 2 * qz * qw, 1 - 2 * qx * qx - 2 * qz * qz, 2 * qy * qz - 2 * qx * qw],
            [2 * qx * qz - 2 * qy * qw, 2 * qy * qz + 2 * qx * qw, 1 - 2 * qx * qx - 2 * qy * qy]
        ])

        return R

    def transform_to_world(self, sensor_id, point):
        """
        将传感器坐标系中的点转换到世界坐标系

        参数:
            sensor_id: 传感器标识符
            point: 传感器坐标系中的点 [x, y, z] 或齐次坐标 [x, y, z, 1]

        返回:
            世界坐标系中的点 [x, y, z]
        """
        if sensor_id not in self.sensors:
            raise ValueError(f"传感器 {sensor_id} 未注册")

        # 确保点是齐次坐标
        if len(point) == 3:
            point_homogeneous = np.array([*point, 1.0])
        else:
            point_homogeneous = np.array(point)

        # 获取传感器的变换矩阵
        transform = self.sensors[sensor_id]['inverse_transform']

        # 执行坐标变换
        world_point_homogeneous = transform @ point_homogeneous

        # 转换回笛卡尔坐标
        world_point = world_point_homogeneous[:3] / world_point_homogeneous[3]

        return world_point

    def transform_multiple_points(self, sensor_id, points):
        """
        批量转换多个点

        参数:
            sensor_id: 传感器标识符
            points: 点的列表，每个点为 [x, y, z]

        返回:
            世界坐标系中的点列表
        """
        return [self.transform_to_world(sensor_id, p) for p in points]

    def get_transform_matrix(self, sensor_id):
        """获取传感器到世界坐标系的变换矩阵"""
        if sensor_id not in self.sensors:
            raise ValueError(f"传感器 {sensor_id} 未注册")

        return self.sensors[sensor_id]['inverse_transform']


# 使用示例
if __name__ == "__main__":
    # 创建坐标系统一类实例
    coordinate_system = SensorCoordinateSystem()
    # 世界坐标系：NDI坐标系
    # 左臂：
    # 1.超声探头相对于机械臂末端
    # 2.机械臂末端相对于机械臂基座
    # 3.机械臂基座相对于NDI坐标系

    # 右臂：
    # 1.穿刺针相对于机械臂末端
    # 2.机械臂末端相对于机械臂基座
    # 3.机械臂基座相对于NDI坐标系

    # 双目相机(右臂)
    # 1.右相机相对于左相机
    # 2.左相机相对于机械臂末端
    # 3.机械臂末端相对于机械臂基座
    # 4.机械臂基座相对于NDI坐标系

    # 力传感器(右臂)
    # 1.力传感器相对于机械臂末端

    # 上述4套子系统即可构成整个标定系统

    # 添加右相机传感器
    right_camera_translation2left_camera_translation = [0.5, 0.2, 1.2]
    right_camera_rotation2left_camera_rotation = [0.707, 0, 0.707, 0]
    coordinate_system.add_sensor("right_camera",
                                 right_camera_translation2left_camera_translation,
                                 right_camera_rotation2left_camera_rotation,parent_frame="left_camera")

    # 添加左相机传感器
    left_camera_translation2tip_translation = [0.5, 0.2, 1.2]
    left_camera_rotation2tip_rotation = [0.707, 0, 0.707, 0]
    coordinate_system.add_sensor("left_camera",
                                 left_camera_translation2tip_translation,
                                 left_camera_rotation2tip_rotation)

    # 示例点（右相机坐标系）
    right_camera_point = [2.0, 3.0, 0.0]

    # 示例点（左相机坐标系）
    left_camera_point = [1.0, -0.5, 2.0]

    # 转换到世界坐标系
    world_point_right_camera = coordinate_system.transform_to_world("right_camera", right_camera_point)
    world_point_left_camera = coordinate_system.transform_to_world("left_camera", left_camera_point)

    print(f"右相机坐标系 ({right_camera_point}) 在世界坐标系中: {world_point_right_camera}")
    print(f"左相机坐标系 ({left_camera_point}) 在世界坐标系中: {world_point_left_camera}")