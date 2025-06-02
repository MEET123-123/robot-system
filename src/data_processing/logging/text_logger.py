import os
import time
from .logger import Logger


class TextLogger(Logger):
    """文本日志记录器"""

    def __init__(self, log_dir=None, log_name=None, separator="\t", timestamp_format="%Y-%m-%d %H:%M:%S.%f"):
        """
        初始化文本日志记录器

        参数:
            log_dir: 日志目录
            log_name: 日志名称
            separator: 字段分隔符
            timestamp_format: 时间戳格式
        """
        super().__init__(log_dir, log_name)
        self.separator = separator
        self.timestamp_format = timestamp_format
        self.file = open(self.log_path, 'w')
        self.header_written = False

    def set_header(self, header):
        """
        设置日志文件头部

        参数:
            header: 头部字段列表
        """
        if not self.header_written:
            header_line = self.separator.join(header) + "\n"
            self.file.write(header_line)
            self.header_written = True

    def log(self, data, timestamp=None):
        """
        记录数据到文本文件

        参数:
            data: 要记录的数据(字典或列表)
            timestamp: 时间戳(可选)
        """
        if timestamp is None:
            timestamp = time.time()

        # 格式化时间戳
        timestamp_str = time.strftime(self.timestamp_format, time.localtime(timestamp))
        timestamp_str = timestamp_str[:-3]  # 保留3位小数

        # 处理数据
        if isinstance(data, dict):
            # 如果是字典，确保按顺序记录
            if not self.header_written:
                self.set_header(["timestamp"] + list(data.keys()))

            values = [str(v) for v in data.values()]
            log_line = self.separator.join([timestamp_str] + values) + "\n"
        elif isinstance(data, list) or isinstance(data, tuple):
            # 如果是列表或元组，直接记录
            values = [str(v) for v in data]
            log_line = self.separator.join([timestamp_str] + values) + "\n"
        else:
            # 如果是单个值，直接记录
            log_line = f"{timestamp_str}{self.separator}{data}\n"

        # 写入文件
        self.file.write(log_line)
        self.file.flush()  # 确保数据写入磁盘

    def close(self):
        """关闭日志文件"""
        if self.file:
            self.file.close()
            self.file = None    