import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .logger import Logger


class ROSLogger(Logger, Node):
    """ROS 2日志记录器"""

    def __init__(self, node_name='ros_logger', topic_name='logging', qos_profile=10):
        """
        初始化ROS日志记录器

        参数:
            node_name: ROS节点名称
            topic_name: 发布的话题名称
            qos_profile: QoS配置
        """
        Logger.__init__(self)
        Node.__init__(self, node_name)

        self.publisher = self.create_publisher(
            String,
            topic_name,
            qos_profile
        )

        self.get_logger().info(f"ROS日志记录器已启动，发布到话题: {topic_name}")

    def log(self, data, timestamp=None):
        """
        发布日志消息

        参数:
            data: 要记录的数据(字符串或可转换为字符串的对象)
            timestamp: 时间戳(可选，ROS日志会自动添加时间戳)
        """
        msg = String()

        if isinstance(data, str):
            msg.data = data
        else:
            try:
                msg.data = str(data)
            except:
                self.get_logger().error("无法将数据转换为字符串")
                return

        self.publisher.publish(msg)

    def close(self):
        """关闭ROS节点"""
        self.destroy_node()