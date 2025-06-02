import os
import time
import csv
from .logger import Logger


class CSVLogger(Logger):
    """CSV日志记录器"""

    def __init__(self, log_dir=None, log_name=None):
        """
        初始化CSV日志记录器

        参数:
            log_dir: 日志目录
            log_name: 日志名称
        """
        if log_name and not log_name.endswith('.csv'):
            log_name += '.csv'

        super().__init__(log_dir, log_name)
        self.file = open(self.log_path, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.header_written = False

    def set_header(self, header):
        """
        设置CSV文件头部

        参数:
            header: 头部字段列表
        """
        if not self.header_written:
            self.writer.writerow(header)
            self.header_written = True

    def log(self, data, timestamp=None):
        """
        记录数据到CSV文件

        参数:
            data: 要记录的数据(字典或列表)
            timestamp: 时间戳(可选)
        """
        if timestamp is None:
            timestamp = time.time()

        # 格式化时间戳
        timestamp_str = time.strftime("%Y-%m-%d %H:%M:%S.%f", time.localtime(timestamp))
        timestamp_str = timestamp_str[:-3]  # 保留3位小数

        # 处理数据
        if isinstance(data, dict):
            # 如果是字典，确保按顺序记录
            if not self.header_written:
                self.set_header(["timestamp"] + list(data.keys()))

            row = [timestamp_str] + [str(v) for v in data.values()]
        elif isinstance(data, list) or isinstance(data, tuple):
            # 如果是列表或元组，直接记录
            row = [timestamp_str] + [str(v) for v in data]
        else:
            # 如果是单个值，直接记录
            row = [timestamp_str, str(data)]

        # 写入CSV
        self.writer.writerow(row)
        self.file.flush()  # 确保数据写入磁盘

    def close(self):
        """关闭CSV文件"""
        if self.file:
            self.file.close()
            self.file = None