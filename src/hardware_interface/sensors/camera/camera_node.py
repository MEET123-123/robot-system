import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from rcl_interfaces.msg import SetParametersResult
from .camera_calibration import CameraCalibration


class CameraNode(Node):
    """手术机器人摄像头接口ROS 2节点"""

    def __init__(self):
        super().__init__('camera_node')

        # 声明参数
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('fps', 30)
        self.declare_parameter('resolution', [640, 480])
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('undistort', False)

        # 获取参数
        self.camera_id = self.get_parameter('camera_id').value
        self.fps = self.get_parameter('fps').value
        self.resolution = self.get_parameter('resolution').value
        self.calibration_file = self.get_parameter('calibration_file').value
        self.undistort = self.get_parameter('undistort').value

        # 注册参数回调
        self.add_on_set_parameters_callback(self.parameters_callback)

        # 初始化摄像头
        self.cap = None
        self.bridge = CvBridge()
        self.calibration = None
        self.initialize_camera()

        # 创建图像发布者
        self.image_pub = self.create_publisher(
            Image,
            'camera/image_raw',
            10
        )

        # 创建定时器
        self.timer = self.create_timer(1.0 / self.fps, self.publish_image)

        self.get_logger().info(f'摄像头节点已启动，ID: {self.camera_id}')

    def initialize_camera(self):
        """初始化摄像头设备"""
        try:
            # 释放已存在的摄像头资源
            if self.cap is not None:
                self.cap.release()

            # 打开摄像头
            self.cap = cv2.VideoCapture(self.camera_id)

            # 设置分辨率
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])

            # 设置帧率
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)

            # 检查摄像头是否成功打开
            if not self.cap.isOpened():
                self.get_logger().error(f"无法打开摄像头 ID: {self.camera_id}")
                return

            # 加载相机校准参数（如果有）
            if self.undistort and self.calibration_file:
                self.calibration = CameraCalibration()
                if not self.calibration.load_calibration(self.calibration_file):
                    self.get_logger().warn("无法加载校准文件，取消图像去畸变")
                    self.undistort = False

            self.get_logger().info(
                f"摄像头已初始化 - 分辨率: {self.resolution[0]}x{self.resolution[1]}, "
                f"帧率: {self.fps} FPS"
            )

        except Exception as e:
            self.get_logger().error(f"摄像头初始化错误: {str(e)}")

    def publish_image(self):
        """捕获并发布图像"""
        if self.cap is None or not self.cap.isOpened():
            return

        try:
            # 读取一帧
            ret, frame = self.cap.read()

            if not ret:
                self.get_logger().warn("无法获取图像帧")
                return

            # 图像去畸变（如果已校准）
            if self.undistort and self.calibration:
                frame = self.calibration.undistort_image(frame)

            # 转换为ROS消息
            img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = "camera_link"

            # 发布图像
            self.image_pub.publish(img_msg)

        except Exception as e:
            self.get_logger().error(f"发布图像时出错: {str(e)}")

    def parameters_callback(self, params):
        """参数变更回调"""
        needs_reinit = False

        for param in params:
            if param.name == 'camera_id' and param.value != self.camera_id:
                self.camera_id = param.value
                needs_reinit = True
            elif param.name == 'fps' and param.value != self.fps:
                self.fps = param.value
                # 更新定时器
                self.timer.cancel()
                self.timer = self.create_timer(1.0 / self.fps, self.publish_image)
            elif param.name == 'resolution' and param.value != self.resolution:
                self.resolution = param.value
                needs_reinit = True
            elif param.name == 'calibration_file' and param.value != self.calibration_file:
                self.calibration_file = param.value
                needs_reinit = True
            elif param.name == 'undistort' and param.value != self.undistort:
                self.undistort = param.value

        # 如果需要，重新初始化摄像头
        if needs_reinit:
            self.initialize_camera()

        return SetParametersResult(successful=True)

    def destroy_node(self):
        """节点销毁时释放资源"""
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()
        self.get_logger().info("摄像头节点已关闭")


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()