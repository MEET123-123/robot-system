import os
import time
import json
from abc import ABC, abstractmethod


class Logger(ABC):
    """日志记录器抽象基类"""

    def __init__(self, log_dir=None, log_name=None):
        """
        初始化日志记录器

        参数:
            log_dir: 日志目录
            log_name: 日志名称
        """
        self.log_dir = log_dir or os.path.join(os.getcwd(), "logs")
        self.log_name = log_name or self._generate_default_log_name()
        self.log_path = os.path.join(self.log_dir, self.log_name)
        self.start_time = time.time()

        # 创建日志目录
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)

    def _generate_default_log_name(self):
        """生成默认日志名称"""
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        return f"log_{timestamp}.txt"

    def get_elapsed_time(self):
        """获取自日志开始以来的时间(秒)"""
        return time.time() - self.start_time

    @abstractmethod
    def log(self, data, timestamp=None):
        """
        记录数据

        参数:
            data: 要记录的数据
            timestamp: 时间戳(可选)
        """
        pass

    @abstractmethod
    def close(self):
        """关闭日志记录器"""
        pass