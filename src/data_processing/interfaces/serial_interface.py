import serial
import time
from .abstract_interface import AbstractInterface


class SerialInterface(AbstractInterface):
    """串口通信接口"""

    def __init__(self, port, baudrate=9600, timeout=1.0, name="SerialInterface"):
        """
        初始化串口接口

        参数:
            port: 串口号
            baudrate: 波特率(默认9600)
            timeout: 超时时间(秒，默认1.0)
            name: 接口名称
        """
        super().__init__(name)
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None

    def connect(self):
        """连接到串口设备"""
        try:
            if self.serial is None or not self.serial.is_open:
                self.serial = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    timeout=self.timeout
                )
                self.connected = True
                self.update_communication_time()
                print(f"成功连接到串口: {self.port}")
                return True
        except Exception as e:
            print(f"连接串口失败: {str(e)}")
            self.connected = False
            return False

    def disconnect(self):
        """断开与串口设备的连接"""
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
                self.connected = False
                print(f"已断开与串口的连接: {self.port}")
                return True
        except Exception as e:
            print(f"断开串口连接失败: {str(e)}")
            return False

    def is_connected(self):
        """检查是否已连接到串口设备"""
        return self.connected and (self.serial is not None) and self.serial.is_open

    def send(self, data):
        """
        发送数据到串口设备

        参数:
            data: 要发送的数据(字符串或字节)
        """
        if not self.is_connected():
            print("未连接到串口设备，无法发送数据")
            return False

        try:
            # 如果数据是字符串，转换为字节
            if isinstance(data, str):
                data = data.encode('utf-8')

            self.serial.write(data)
            self.update_communication_time()
            return True
        except Exception as e:
            print(f"发送数据失败: {str(e)}")
            self.connected = False
            return False

    def receive(self):
        """
        从串口设备接收数据

        返回:
            接收到的数据(字节)
        """
        if not self.is_connected():
            print("未连接到串口设备，无法接收数据")
            return None

        try:
            if self.serial.in_waiting > 0:
                data = self.serial.read(self.serial.in_waiting)
                self.update_communication_time()
                return data
            return None
        except Exception as e:
            print(f"接收数据失败: {str(e)}")
            self.connected = False
            return None