import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .abstract_interface import AbstractInterface


class ROSInterface(AbstractInterface, Node):
    """ROS 2通信接口"""

    def __init__(self, node_name="ros_interface", topic_name="communication", qos_profile=10, name="ROSInterface"):
        """
        初始化ROS接口

        参数:
            node_name: ROS节点名称
            topic_name: 通信话题名称
            qos_profile: QoS配置
            name: 接口名称
        """
        AbstractInterface.__init__(self, name)
        Node.__init__(self, node_name)

        self.topic_name = topic_name
        self.publisher = self.create_publisher(String, topic_name, qos_profile)
        self.subscription = self.create_subscription(
            String,
            topic_name,
            self._message_callback,
            qos_profile
        )

        self.last_received_message = None
        self.get_logger().info(f"ROS接口已初始化，话题: {topic_name}")

    def _message_callback(self, msg):
        """消息回调函数"""
        self.last_received_message = msg.data
        self.update_communication_time()

    def connect(self):
        """连接到ROS网络"""
        # ROS节点在初始化时已经连接
        self.connected = True
        self.get_logger().info("已连接到ROS网络")
        return True

    def disconnect(self):
        """断开与ROS网络的连接"""
        self.connected = False
        self.destroy_node()
        self.get_logger().info("已断开与ROS网络的连接")

    def is_connected(self):
        """检查是否已连接到ROS网络"""
        return self.connected

    def send(self, data):
        """
        发布消息到ROS话题

        参数:
            data: 要发送的数据(字符串)
        """
        if not self.is_connected():
            self.get_logger().error("未连接到ROS网络，无法发送数据")
            return False

        msg = String()
        msg.data = str(data)
        self.publisher.publish(msg)
        self.update_communication_time()
        return True

    def receive(self):
        """
        获取最后接收到的ROS消息

        返回:
            接收到的消息(字符串)或None
        """
        return self.last_received_message