#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import threading
import time
from enum import Enum
from . import api_bridge  # NDI API封装层


class NDITrackerNode(Node):
    """NDI红外跟踪系统ROS 2节点"""

    class TrackingStatus(Enum):
        NOT_TRACKING = 0
        TRACKING = 1
        LOST = 2

    def __init__(self):
        super().__init__('ndi_tracker_node')

        # 声明参数
        self.declare_parameter('ndi_ip', '127.0.0.1')
        self.declare_parameter('ndi_port', 8765)
        self.declare_parameter('tracking_rate', 60.0)  # Hz
        self.declare_parameter('tools', [])  # 要跟踪的工具列表
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('parent_frame', 'world')

        # 获取参数
        self.ndi_ip = self.get_parameter('ndi_ip').value
        self.ndi_port = self.get_parameter('ndi_port').value
        self.tracking_rate = self.get_parameter('tracking_rate').value
        self.tool_list = self.get_parameter('tools').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.parent_frame = self.get_parameter('parent_frame').value

        # 初始化NDI接口
        self.ndi_api = api_bridge.NDIApiBridge()
        self.tracking_status = {tool: self.TrackingStatus.NOT_TRACKING for tool in self.tool_list}
        self.tool_poses = {tool: None for tool in self.tool_list}
        self.tracking_thread = None
        self.is_tracking = False

        # 创建发布者
        self.pose_publishers = {
            tool: self.create_publisher(PoseStamped, f'ndi/{tool}/pose', 10)
            for tool in self.tool_list
        }

        # TF广播器
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        # 连接到NDI系统
        self.connect_to_ndi()

        # 启动跟踪线程
        self.start_tracking()

        self.get_logger().info(f'NDI跟踪系统节点已启动，跟踪工具: {self.tool_list}')

    def connect_to_ndi(self):
        """连接到NDI跟踪系统"""
        try:
            self.get_logger().info(f"正在连接到NDI系统: {self.ndi_ip}:{self.ndi_port}")

            # 连接到NDI系统
            if not self.ndi_api.connect(self.ndi_ip, self.ndi_port):
                self.get_logger().error("无法连接到NDI系统")
                return False

            # 初始化跟踪系统
            if not self.ndi_api.initialize():
                self.get_logger().error("NDI系统初始化失败")
                self.ndi_api.disconnect()
                return False

            # 加载工具定义
            for tool in self.tool_list:
                if not self.ndi_api.load_tool(tool):
                    self.get_logger().warn(f"无法加载工具: {tool}")

            self.get_logger().info("成功连接到NDI系统")
            return True

        except Exception as e:
            self.get_logger().error(f"连接NDI系统时出错: {str(e)}")
            return False

    def start_tracking(self):
        """开始跟踪线程"""
        if self.is_tracking:
            return

        self.is_tracking = True
        self.tracking_thread = threading.Thread(target=self._tracking_loop)
        self.tracking_thread.daemon = True
        self.tracking_thread.start()

    def stop_tracking(self):
        """停止跟踪线程"""
        self.is_tracking = False
        if self.tracking_thread and self.tracking_thread.is_alive():
            self.tracking_thread.join(timeout=1.0)

    def _tracking_loop(self):
        """跟踪主循环"""
        rate = self.create_rate(self.tracking_rate)

        while self.is_tracking and rclpy.ok():
            try:
                # 更新所有工具的位姿
                for tool in self.tool_list:
                    success, pose = self.ndi_api.get_tool_pose(tool)

                    if success:
                        self.tool_poses[tool] = pose
                        self.tracking_status[tool] = self.TrackingStatus.TRACKING

                        # 发布位姿消息
                        self._publish_pose(tool, pose)

                        # 发布TF变换
                        if self.publish_tf:
                            self._publish_tf(tool, pose)
                    else:
                        self.tracking_status[tool] = self.TrackingStatus.LOST
                        self.get_logger().debug(f"工具 {tool} 跟踪丢失")

                rate.sleep()

            except Exception as e:
                self.get_logger().error(f"跟踪循环出错: {str(e)}")
                time.sleep(0.1)  # 出错时等待一段时间

    def _publish_pose(self, tool, pose):
        """发布工具位姿消息"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.parent_frame

        # 转换位姿数据
        msg.pose.position.x = pose[0]  # X
        msg.pose.position.y = pose[1]  # Y
        msg.pose.position.z = pose[2]  # Z

        # 四元数表示姿态 (从旋转矩阵转换)
        rotation_matrix = self._position_to_rotation_matrix(pose)
        quaternion = self._rotation_matrix_to_quaternion(rotation_matrix)

        msg.pose.orientation.x = quaternion[0]
        msg.pose.orientation.y = quaternion[1]
        msg.pose.orientation.z = quaternion[2]
        msg.pose.orientation.w = quaternion[3]

        # 发布消息
        self.pose_publishers[tool].publish(msg)

    def _publish_tf(self, tool, pose):
        """发布TF变换"""
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.parent_frame
        tf_msg.child_frame_id = f'ndi_{tool}'

        # 设置位置
        tf_msg.transform.translation.x = pose[0]
        tf_msg.transform.translation.y = pose[1]
        tf_msg.transform.translation.z = pose[2]

        # 设置姿态
        rotation_matrix = self._position_to_rotation_matrix(pose)
        quaternion = self._rotation_matrix_to_quaternion(rotation_matrix)

        tf_msg.transform.rotation.x = quaternion[0]
        tf_msg.transform.rotation.y = quaternion[1]
        tf_msg.transform.rotation.z = quaternion[2]
        tf_msg.transform.rotation.w = quaternion[3]

        # 广播变换
        self.tf_broadcaster.sendTransform(tf_msg)

    def _position_to_rotation_matrix(self, pose):
        """从位置数据提取旋转矩阵（假设pose包含旋转信息）"""
        # 这里需要根据NDI API返回的数据格式进行调整
        # 假设pose是 [x, y, z, rx, ry, rz] 或包含旋转矩阵信息
        # 简化实现，实际应根据NDI数据格式调整

        # 创建单位矩阵
        rotation_matrix = np.eye(3)

        # 如果pose包含欧拉角，需要转换为旋转矩阵
        # 这里只是示例，实际实现需要根据NDI数据格式调整
        if len(pose) >= 6:
            rx, ry, rz = pose[3:6]  # 假设是欧拉角

            # 绕X轴旋转
            Rx = np.array([
                [1, 0, 0],
                [0, np.cos(rx), -np.sin(rx)],
                [0, np.sin(rx), np.cos(rx)]
            ])

            # 绕Y轴旋转
            Ry = np.array([
                [np.cos(ry), 0, np.sin(ry)],
                [0, 1, 0],
                [-np.sin(ry), 0, np.cos(ry)]
            ])

            # 绕Z轴旋转
            Rz = np.array([
                [np.cos(rz), -np.sin(rz), 0],
                [np.sin(rz), np.cos(rz), 0],
                [0, 0, 1]
            ])

            # 组合旋转矩阵 (顺序可能需要根据实际情况调整)
            rotation_matrix = Rz @ Ry @ Rx

        return rotation_matrix

    def _rotation_matrix_to_quaternion(self, rotation_matrix):
        """将旋转矩阵转换为四元数"""
        # 简化实现，实际应使用更健壮的算法
        m = rotation_matrix
        tr = m[0, 0] + m[1, 1] + m[2, 2]

        if tr > 0:
            S = np.sqrt(tr + 1.0) * 2
            qw = 0.25 * S
            qx = (m[2, 1] - m[1, 2]) / S
            qy = (m[0, 2] - m[2, 0]) / S
            qz = (m[1, 0] - m[0, 1]) / S
        elif (m[0, 0] > m[1, 1]) and (m[0, 0] > m[2, 2]):
            S = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
            qw = (m[2, 1] - m[1, 2]) / S
            qx = 0.25 * S
            qy = (m[0, 1] + m[1, 0]) / S
            qz = (m[0, 2] + m[2, 0]) / S
        elif m[1, 1] > m[2, 2]:
            S = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
            qw = (m[0, 2] - m[2, 0]) / S
            qx = (m[0, 1] + m[1, 0]) / S
            qy = 0.25 * S
            qz = (m[1, 2] + m[2, 1]) / S
        else:
            S = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
            qw = (m[1, 0] - m[0, 1]) / S
            qx = (m[0, 2] + m[2, 0]) / S
            qy = (m[1, 2] + m[2, 1]) / S
            qz = 0.25 * S

        return [qx, qy, qz, qw]

    def destroy_node(self):
        """节点销毁时释放资源"""
        self.stop_tracking()

        if self.ndi_api:
            self.ndi_api.disconnect()

        super().destroy_node()
        self.get_logger().info("NDI跟踪系统节点已关闭")


def main(args=None):
    rclpy.init(args=args)
    node = NDITrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()