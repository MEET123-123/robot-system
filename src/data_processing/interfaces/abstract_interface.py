from abc import ABC, abstractmethod
import time


class AbstractInterface(ABC):
    """硬件接口抽象基类"""

    def __init__(self, name):
        """
        初始化硬件接口

        参数:
            name: 接口名称
        """
        self.name = name
        self.connected = False
        self.last_communication_time = 0

    @abstractmethod
    def connect(self):
        """连接到硬件设备"""
        pass

    @abstractmethod
    def disconnect(self):
        """断开与硬件设备的连接"""
        pass

    @abstractmethod
    def is_connected(self):
        """检查是否已连接到硬件设备"""
        pass

    @abstractmethod
    def send(self, data):
        """
        发送数据到硬件设备

        参数:
            data: 要发送的数据
        """
        pass

    @abstractmethod
    def receive(self):
        """
        从硬件设备接收数据

        返回:
            接收到的数据
        """
        pass

    def update_communication_time(self):
        """更新最后通信时间"""
        self.last_communication_time = time.time()

    def get_time_since_last_communication(self):
        """获取自上次通信以来的时间(秒)"""
        return time.time() - self.last_communication_time