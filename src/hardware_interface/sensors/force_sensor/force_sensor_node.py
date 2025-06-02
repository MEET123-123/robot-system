#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64MultiArray, Bool
from rcl_interfaces.msg import SetParametersResult
import numpy as np
import time
import threading
from enum import Enum


class ForceSensorNode(Node):
    """手术机器人力传感器ROS 2节点"""

    class SensorType(Enum):
        F / T_SENSOR = 1  # 六维力/力矩传感器
        PRESSURE_SENSOR = 2  # 压力传感器
        FORCE_TORQUE_SENSOR = 3  # 力/扭矩传感器

    def __init__(self):
        super().__init__('force_sensor_node')

        # 声明参数
        self.declare_parameter('sensor_type', 'F/T_SENSOR')
        self.declare_parameter('sensor_id', 0)
        self.declare_parameter('frame_id', 'force_sensor_link')
        self.declare_parameter('sampling_rate', 100.0)  # Hz
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('filter_window_size', 5)
        self.declare_parameter('force_threshold', [5.0, 5.0, 5.0])  # N
        self.declare_parameter('torque_threshold', [0.5, 0.5, 0.5])  # Nm

        # 获取参数
        self.sensor_type = self.get_parameter('sensor_type').value
        self.sensor_id = self.get_parameter('sensor_id').value
        self.frame_id = self.get_parameter('frame_id').value
        self.sampling_rate = self.get_parameter('sampling_rate').value
        self.calibration_file = self.get_parameter('calibration_file').value
        self.filter_window_size = self.get_parameter('filter_window_size').value
        self.force_threshold = np.array(self.get_parameter('force_threshold').value)
        self.torque_threshold = np.array(self.get_parameter('torque_threshold').value)

        # 注册参数回调
        self.add_on_set_parameters_callback(self.parameters_callback)

        # 初始化传感器
        self.sensor = None
        self.initialized = False
        self.force_data = np.zeros(6)  # [Fx, Fy, Fz, Mx, My, Mz]
        self.raw_data_buffer = []
        self.filtered_data = np.zeros(6)
        self.emergency_stop = False

        # 创建发布者
        self.force_pub = self.create_publisher(
            Float64MultiArray,
            'force_sensor/data',
            10
        )

        self.emergency_pub = self.create_publisher(
            Bool,
            'force_sensor/emergency_stop',
            10
        )

        # 创建定时器
        self.sampling_timer = self.create_timer(
            1.0 / self.sampling_rate,
            self.sample_data
        )

        # 初始化传感器
        self.initialize_sensor()

        self.get_logger().info(f'力传感器节点已启动，类型: {self.sensor_type}')

    def initialize_sensor(self):
        """初始化力传感器"""
        try:
            # 释放已存在的传感器资源
            if self.sensor is not None:
                self.sensor.close()

            # 根据传感器类型创建相应的驱动
            if self.sensor_type == 'F/T_SENSOR':
                # 这里应该是实际的传感器驱动，这里用模拟代替
                self.sensor = self._create_ft_sensor_driver()
            elif self.sensor_type == 'PRESSURE_SENSOR':
                self.sensor = self._create_pressure_sensor_driver()
            elif self.sensor_type == 'FORCE_TORQUE_SENSOR':
                self.sensor = self._create_force_torque_sensor_driver()
            else:
                self.get_logger().error(f"未知的传感器类型: {self.sensor_type}")
                return

            # 连接传感器
            if not self.sensor.connect(self.sensor_id):
                self.get_logger().error(f"无法连接到传感器 ID: {self.sensor_id}")
                return

            # 加载校准数据
            if self.calibration_file:
                if not self.sensor.load_calibration(self.calibration_file):
                    self.get_logger().warn("无法加载校准文件，使用默认校准")

            # 初始化成功
            self.initialized = True
            self.get_logger().info(f"传感器已成功初始化 - 采样率: {self.sampling_rate} Hz")

        except Exception as e:
            self.get_logger().error(f"传感器初始化错误: {str(e)}")
            self.initialized = False

    def _create_ft_sensor_driver(self):
        """创建六维力/力矩传感器驱动（实际应用中替换为真实驱动）"""
        # 这里使用模拟驱动，实际应用中应替换为真实的传感器驱动
        return SimulatedFTSensorDriver()

    def _create_pressure_sensor_driver(self):
        """创建压力传感器驱动（实际应用中替换为真实驱动）"""
        return SimulatedPressureSensorDriver()

    def _create_force_torque_sensor_driver(self):
        """创建力/扭矩传感器驱动（实际应用中替换为真实驱动）"""
        return SimulatedForceTorqueSensorDriver()

    def sample_data(self):
        """采样传感器数据"""
        if not self.initialized or self.emergency_stop:
            return

        try:
            # 读取传感器数据
            success, data = self.sensor.read_data()

            if not success:
                self.get_logger().warn("无法读取传感器数据")
                return

            # 更新原始数据缓冲区
            self.raw_data_buffer.append(data)
            if len(self.raw_data_buffer) > self.filter_window_size:
                self.raw_data_buffer.pop(0)

            # 应用滤波
            self.filtered_data = self._apply_filter()

            # 检查阈值
            self._check_thresholds()

            # 发布数据
            self._publish_data()

        except Exception as e:
            self.get_logger().error(f"采样数据时出错: {str(e)}")

    def _apply_filter(self):
        """应用移动平均滤波"""
        if not self.raw_data_buffer:
            return np.zeros(6)

        # 计算平均值
        return np.mean(np.array(self.raw_data_buffer), axis=0)

    def _check_thresholds(self):
        """检查力/力矩是否超过阈值"""
        force = self.filtered_data[:3]
        torque = self.filtered_data[3:]

        # 检查力阈值
        force_exceeded = np.any(np.abs(force) > self.force_threshold)

        # 检查力矩阈值
        torque_exceeded = np.any(np.abs(torque) > self.torque_threshold)

        # 如果超过阈值，触发紧急停止
        if force_exceeded or torque_exceeded:
            self.emergency_stop = True
            self.get_logger().warn("力/力矩超过阈值，触发紧急停止!")

            # 发布紧急停止信号
            emergency_msg = Bool()
            emergency_msg.data = True
            self.emergency_pub.publish(emergency_msg)

    def _publish_data(self):
        """发布传感器数据"""
        # 创建消息
        force_msg = Float64MultiArray()
        force_msg.data = self.filtered_data.tolist()

        # 发布消息
        self.force_pub.publish(force_msg)

    def reset_emergency_stop(self):
        """重置紧急停止状态"""
        if self.emergency_stop:
            self.emergency_stop = False
            emergency_msg = Bool()
            emergency_msg.data = False
            self.emergency_pub.publish(emergency_msg)
            self.get_logger().info("紧急停止状态已重置")

    def parameters_callback(self, params):
        """参数变更回调"""
        needs_reinit = False

        for param in params:
            if param.name == 'sensor_type' and param.value != self.sensor_type:
                self.sensor_type = param.value
                needs_reinit = True
            elif param.name == 'sensor_id' and param.value != self.sensor_id:
                self.sensor_id = param.value
                needs_reinit = True
            elif param.name == 'sampling_rate' and param.value != self.sampling_rate:
                self.sampling_rate = param.value
                # 更新定时器
                self.sampling_timer.cancel()
                self.sampling_timer = self.create_timer(
                    1.0 / self.sampling_rate,
                    self.sample_data
                )
            elif param.name == 'calibration_file' and param.value != self.calibration_file:
                self.calibration_file = param.value
                needs_reinit = True
            elif param.name == 'filter_window_size' and param.value != self.filter_window_size:
                self.filter_window_size = param.value
            elif param.name == 'force_threshold' and param.value != self.force_threshold.tolist():
                self.force_threshold = np.array(param.value)
            elif param.name == 'torque_threshold' and param.value != self.torque_threshold.tolist():
                self.torque_threshold = np.array(param.value)

        # 如果需要，重新初始化传感器
        if needs_reinit:
            self.initialize_sensor()

        return SetParametersResult(successful=True)

    def destroy_node(self):
        """节点销毁时释放资源"""
        if self.sensor is not None:
            self.sensor.close()
        super().destroy_node()
        self.get_logger().info("力传感器节点已关闭")


# 模拟传感器驱动（实际应用中替换为真实驱动）
class SimulatedFTSensorDriver:
    """模拟六维力/力矩传感器驱动"""

    def __init__(self):
        self.connected = False
        self.calibration_loaded = False

    def connect(self, sensor_id):
        """连接传感器"""
        # 模拟连接过程
        time.sleep(0.5)
        self.connected = True
        return True

    def load_calibration(self, calibration_file):
        """加载校准文件"""
        # 模拟加载校准文件
        time.sleep(0.2)
        self.calibration_loaded = True
        return True

    def read_data(self):
        """读取传感器数据"""
        if not self.connected:
            return False, np.zeros(6)

        # 模拟六维力数据 (Fx, Fy, Fz, Mx, My, Mz)
        # 实际应用中应替换为从真实传感器读取数据
        force = np.random.normal(0, 0.1, 6)

        # 添加一些静态偏置来模拟真实传感器
        bias = np.array([0.1, 0.05, 0.2, 0.01, 0.02, 0.03])
        force += bias

        return True, force

    def close(self):
        """关闭传感器连接"""
        self.connected = False


class SimulatedPressureSensorDriver:
    """模拟压力传感器驱动"""

    def __init__(self):
        self.connected = False

    def connect(self, sensor_id):
        time.sleep(0.3)
        self.connected = True
        return True

    def load_calibration(self, calibration_file):
        return True

    def read_data(self):
        if not self.connected:
            return False, np.zeros(6)

        # 模拟压力传感器数据
        pressure = np.random.normal(1.0, 0.05, 1)
        return True, np.array([pressure[0], 0, 0, 0, 0, 0])

    def close(self):
        self.connected = False


class SimulatedForceTorqueSensorDriver:
    """模拟力/扭矩传感器驱动"""

    def __init__(self):
        self.connected = False

    def connect(self, sensor_id):
        time.sleep(0.3)
        self.connected = True
        return True

    def load_calibration(self, calibration_file):
        return True

    def read_data(self):
        if not self.connected:
            return False, np.zeros(6)

        # 模拟力/扭矩传感器数据
        force = np.random.normal(0.5, 0.05, 3)
        torque = np.random.normal(0.05, 0.01, 3)
        return True, np.concatenate((force, torque))

    def close(self):
        self.connected = False


def main(args=None):
    rclpy.init(args=args)
    node = ForceSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()