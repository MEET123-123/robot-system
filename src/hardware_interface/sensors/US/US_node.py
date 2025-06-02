#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from rcl_interfaces.msg import SetParametersResult
from .ultrasound_probe import UltrasoundProbe
from .image_processing import UltrasoundImageProcessor


class UltrasoundNode(Node):
    """手术机器人超声图像ROS 2节点"""

    def __init__(self):
        super().__init__('ultrasound_node')

        # 声明参数
        self.declare_parameter('probe_model', 'default')
        self.declare_parameter('device_id', 0)
        self.declare_parameter('frame_id', 'ultrasound_frame')
        self.declare_parameter('resolution', [640, 480])
        self.declare_parameter('frequency', 5.0)  # MHz
        self.declare_parameter('gain', 50.0)  # %
        self.declare_parameter('depth', 15.0)  # cm
        self.declare_parameter('process_image', True)
        self.declare_parameter('fps', 30.0)

        # 获取参数
        self.probe_model = self.get_parameter('probe_model').value
        self.device_id = self.get_parameter('device_id').value
        self.frame_id = self.get_parameter('frame_id').value
        self.resolution = self.get_parameter('resolution').value
        self.frequency = self.get_parameter('frequency').value
        self.gain = self.get_parameter('gain').value
        self.depth = self.get_parameter('depth').value
        self.process_image = self.get_parameter('process_image').value
        self.fps = self.get_parameter('fps').value

        # 注册参数回调
        self.add_on_set_parameters_callback(self.parameters_callback)

        # 初始化超声探头和图像处理
        self.bridge = CvBridge()
        self.probe = UltrasoundProbe(self.probe_model)
        self.image_processor = UltrasoundImageProcessor()

        # 创建发布者
        self.image_pub = self.create_publisher(
            Image,
            'ultrasound/image_raw',
            10
        )

        self.processed_image_pub = self.create_publisher(
            Image,
            'ultrasound/image_processed',
            10
        )

        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            'ultrasound/camera_info',
            10
        )

        # 初始化设备
        self.initialize_device()

        # 创建定时器
        self.timer = self.create_timer(1.0 / self.fps, self.publish_ultrasound_data)

        self.get_logger().info(f'超声图像节点已启动，型号: {self.probe_model}')

    def initialize_device(self):
        """初始化超声设备"""
        try:
            # 连接到超声探头
            if not self.probe.connect(self.device_id):
                self.get_logger().error(f"无法连接到超声探头 ID: {self.device_id}")
                return

            # 配置探头参数
            if not self.probe.configure(
                    resolution=self.resolution,
                    frequency=self.frequency,
                    gain=self.gain,
                    depth=self.depth
            ):
                self.get_logger().error("超声探头配置失败")
                return

            # 获取相机信息
            self.camera_info = self._create_camera_info()

            self.get_logger().info("超声设备初始化成功")

        except Exception as e:
            self.get_logger().error(f"初始化超声设备时出错: {str(e)}")

    def publish_ultrasound_data(self):
        """发布超声图像数据"""
        try:
            # 获取原始超声图像
            success, raw_image = self.probe.capture_image()

            if not success:
                self.get_logger().warn("无法获取超声图像")
                return

            # 转换为ROS消息
            img_msg = self.bridge.cv2_to_imgmsg(raw_image, "mono8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = self.frame_id

            # 发布原始图像
            self.image_pub.publish(img_msg)

            # 图像处理（如果启用）
            if self.process_image:
                processed_image = self.image_processor.process(raw_image)
                processed_msg = self.bridge.cv2_to_imgmsg(processed_image, "mono8")
                processed_msg.header = img_msg.header

                # 发布处理后的图像
                self.processed_image_pub.publish(processed_msg)

            # 发布相机信息
            self.camera_info.header = img_msg.header
            self.camera_info_pub.publish(self.camera_info)

        except Exception as e:
            self.get_logger().error(f"发布超声数据时出错: {str(e)}")

    def _create_camera_info(self):
        """创建相机信息"""
        info = CameraInfo()
        info.width = self.resolution[0]
        info.height = self.resolution[1]

        # 设置相机内参（简化，实际应根据超声探头校准）
        cx = info.width / 2.0
        cy = info.height / 2.0
        fx = 500.0  # 简化的焦距
        fy = 500.0

        info.k = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        info.d = [0, 0, 0, 0, 0]  # 无畸变
        info.r = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        info.p = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]

        return info

    def parameters_callback(self, params):
        """参数变更回调"""
        needs_reconfigure = False

        for param in params:
            if param.name == 'probe_model' and param.value != self.probe_model:
                self.probe_model = param.value
                needs_reconfigure = True
            elif param.name == 'device_id' and param.value != self.device_id:
                self.device_id = param.value
                needs_reconfigure = True
            elif param.name == 'resolution' and param.value != self.resolution:
                self.resolution = param.value
                needs_reconfigure = True
            elif param.name == 'frequency' and param.value != self.frequency:
                self.frequency = param.value
                needs_reconfigure = True
            elif param.name == 'gain' and param.value != self.gain:
                self.gain = param.value
                needs_reconfigure = True
            elif param.name == 'depth' and param.value != self.depth:
                self.depth = param.value
                needs_reconfigure = True
            elif param.name == 'process_image' and param.value != self.process_image:
                self.process_image = param.value
            elif param.name == 'fps' and param.value != self.fps:
                self.fps = param.value
                # 更新定时器
                self.timer.cancel()
                self.timer = self.create_timer(1.0 / self.fps, self.publish_ultrasound_data)

        # 如果需要，重新配置探头
        if needs_reconfigure:
            self.initialize_device()

        return SetParametersResult(successful=True)

    def destroy_node(self):
        """节点销毁时释放资源"""
        if self.probe:
            self.probe.disconnect()
        super().destroy_node()
        self.get_logger().info("超声图像节点已关闭")


def main(args=None):
    rclpy.init(args=args)
    node = UltrasoundNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()