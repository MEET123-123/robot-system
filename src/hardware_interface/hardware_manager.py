#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger, SetBool
from hardware_interface.arm.robot_arm import RobotArm
from hardware_interface.sensors.camera.camera_node import CameraNode
from hardware_interface.sensors.force_sensor.force_sensor_node import ForceSensorNode
from hardware_interface.sensors.ndi.ndi_tracker import NDITrackerNode
from hardware_interface.sensors.ultrasound.ultrasound_node import UltrasoundNode
from hardware_interface.gripper.gripper import Gripper
from hardware_interface.light.light import LightSystem


class HardwareManager(Node):
    """手术机器人硬件管理器"""

    def __init__(self):
        super().__init__('hardware_manager')

        # 创建回调组
        self.callback_group = ReentrantCallbackGroup()

        # 初始化硬件组件
        self.robot_arm = None
        self.camera = None
        self.force_sensor = None
        self.ndi_tracker = None
        self.ultrasound = None
        self.gripper = None
        self.light_system = None

        # 声明参数
        self.declare_parameter('arm_enabled', True)
        self.declare_parameter('camera_enabled', True)
        self.declare_parameter('force_sensor_enabled', True)
        self.declare_parameter('ndi_tracker_enabled', True)
        self.declare_parameter('ultrasound_enabled', True)
        self.declare_parameter('gripper_enabled', True)
        self.declare_parameter('light_system_enabled', True)

        # 创建服务
        self.start_all_srv = self.create_service(
            Trigger,
            'hardware/start_all',
            self.start_all_callback,
            callback_group=self.callback_group
        )

        self.stop_all_srv = self.create_service(
            Trigger,
            'hardware/stop_all',
            self.stop_all_callback,
            callback_group=self.callback_group
        )

        self.reset_all_srv = self.create_service(
            Trigger,
            'hardware/reset_all',
            self.reset_all_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info("硬件管理器已启动")

        # 初始化硬件
        self.initialize_hardware()

    def initialize_hardware(self):
        """初始化所有硬件组件"""
        try:
            # 获取参数
            arm_enabled = self.get_parameter('arm_enabled').value
            camera_enabled = self.get_parameter('camera_enabled').value
            force_sensor_enabled = self.get_parameter('force_sensor_enabled').value
            ndi_tracker_enabled = self.get_parameter('ndi_tracker_enabled').value
            ultrasound_enabled = self.get_parameter('ultrasound_enabled').value
            gripper_enabled = self.get_parameter('gripper_enabled').value
            light_system_enabled = self.get_parameter('light_system_enabled').value

            # 初始化机械臂
            if arm_enabled:
                self.robot_arm = RobotArm()
                self.get_logger().info("机械臂已初始化")

            # 初始化摄像头
            if camera_enabled:
                self.camera = CameraNode()
                self.get_logger().info("摄像头已初始化")

            # 初始化力传感器
            if force_sensor_enabled:
                self.force_sensor = ForceSensorNode()
                self.get_logger().info("力传感器已初始化")

            # 初始化NDI跟踪系统
            if ndi_tracker_enabled:
                self.ndi_tracker = NDITrackerNode()
                self.get_logger().info("NDI跟踪系统已初始化")

            # 初始化超声设备
            if ultrasound_enabled:
                self.ultrasound = UltrasoundNode()
                self.get_logger().info("超声设备已初始化")

            # 初始化夹爪
            if gripper_enabled:
                self.gripper = Gripper()
                self.get_logger().info("夹爪已初始化")

            # 初始化照明系统
            if light_system_enabled:
                self.light_system = LightSystem()
                self.get_logger().info("照明系统已初始化")

            self.get_logger().info("所有硬件组件初始化完成")

        except Exception as e:
            self.get_logger().error(f"硬件初始化失败: {str(e)}")

    def start_all_callback(self, request, response):
        """启动所有硬件组件的服务回调"""
        self.get_logger().info("收到启动所有硬件的请求")

        try:
            # 启动各个硬件组件
            if self.robot_arm:
                self.robot_arm.start()

            if self.camera:
                rclpy.spin_once(self.camera, timeout_sec=0.1)

            if self.force_sensor:
                rclpy.spin_once(self.force_sensor, timeout_sec=0.1)

            if self.ndi_tracker:
                rclpy.spin_once(self.ndi_tracker, timeout_sec=0.1)

            if self.ultrasound:
                rclpy.spin_once(self.ultrasound, timeout_sec=0.1)

            if self.gripper:
                self.gripper.activate()

            if self.light_system:
                self.light_system.turn_on()

            response.success = True
            response.message = "所有硬件组件已启动"
            self.get_logger().info("所有硬件组件已启动")

        except Exception as e:
            response.success = False
            response.message = f"启动硬件失败: {str(e)}"
            self.get_logger().error(f"启动硬件失败: {str(e)}")

        return response

    def stop_all_callback(self, request, response):
        """停止所有硬件组件的服务回调"""
        self.get_logger().info("收到停止所有硬件的请求")

        try:
            # 停止各个硬件组件
            if self.robot_arm:
                self.robot_arm.stop()

            if self.gripper:
                self.gripper.deactivate()

            if self.light_system:
                self.light_system.turn_off()

            response.success = True
            response.message = "所有硬件组件已停止"
            self.get_logger().info("所有硬件组件已停止")

        except Exception as e:
            response.success = False
            response.message = f"停止硬件失败: {str(e)}"
            self.get_logger().error(f"停止硬件失败: {str(e)}")

        return response

    def reset_all_callback(self, request, response):
        """重置所有硬件组件的服务回调"""
        self.get_logger().info("收到重置所有硬件的请求")

        try:
            # 停止所有硬件
            self.stop_all_callback(request, response)

            # 重新初始化硬件
            self.initialize_hardware()

            # 启动所有硬件
            self.start_all_callback(request, response)

            response.success = True
            response.message = "所有硬件组件已重置"
            self.get_logger().info("所有硬件组件已重置")

        except Exception as e:
            response.success = False
            response.message = f"重置硬件失败: {str(e)}"
            self.get_logger().error(f"重置硬件失败: {str(e)}")

        return response

    def get_hardware_status(self):
        """获取所有硬件组件的状态"""
        status = {
            'robot_arm': self.robot_arm.get_status() if self.robot_arm else "未初始化",
            'camera': "运行中" if self.camera else "未初始化",
            'force_sensor': "运行中" if self.force_sensor else "未初始化",
            'ndi_tracker': "运行中" if self.ndi_tracker else "未初始化",
            'ultrasound': "运行中" if self.ultrasound else "未初始化",
            'gripper': self.gripper.get_status() if self.gripper else "未初始化",
            'light_system': self.light_system.get_status() if self.light_system else "未初始化"
        }

        return status

    def destroy_node(self):
        """节点销毁时释放资源"""
        # 停止所有硬件
        req = Trigger.Request()
        self.stop_all_callback(req, Trigger.Response())

        # 销毁各个节点
        if self.camera:
            self.camera.destroy_node()

        if self.force_sensor:
            self.force_sensor.destroy_node()

        if self.ndi_tracker:
            self.ndi_tracker.destroy_node()

        if self.ultrasound:
            self.ultrasound.destroy_node()

        super().destroy_node()
        self.get_logger().info("硬件管理器已关闭")


def main(args=None):
    rclpy.init(args=args)

    # 创建硬件管理器节点
    hardware_manager = HardwareManager()

    # 创建多线程执行器
    executor = MultiThreadedExecutor()

    # 添加硬件管理器节点
    executor.add_node(hardware_manager)

    # 添加其他硬件节点
    if hardware_manager.camera:
        executor.add_node(hardware_manager.camera)

    if hardware_manager.force_sensor:
        executor.add_node(hardware_manager.force_sensor)

    if hardware_manager.ndi_tracker:
        executor.add_node(hardware_manager.ndi_tracker)

    if hardware_manager.ultrasound:
        executor.add_node(hardware_manager.ultrasound)

    try:
        # 开始旋转执行器
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # 关闭执行器和节点
        executor.shutdown()
        hardware_manager.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()