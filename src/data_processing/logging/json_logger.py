import os
import time
import json
from .logger import Logger


class JSONLogger(Logger):
    """JSON日志记录器"""

    def __init__(self, log_dir=None, log_name=None, indent=None):
        """
        初始化JSON日志记录器

        参数:
            log_dir: 日志目录
            log_name: 日志名称
            indent: JSON缩进(None表示不缩进)
        """
        if log_name and not log_name.endswith('.json'):
            log_name += '.json'

        super().__init__(log_dir, log_name)
        self.indent = indent
        self.logs = []

    def log(self, data, timestamp=None):
        """
        记录数据到JSON

        参数:
            data: 要记录的数据(字典)
            timestamp: 时间戳(可选)
        """
        if timestamp is None:
            timestamp = time.time()

        # 格式化时间戳
        timestamp_str = time.strftime("%Y-%m-%d %H:%M:%S.%f", time.localtime(timestamp))
        timestamp_str = timestamp_str[:-3]  # 保留3位小数

        # 创建日志条目
        log_entry = {
            'timestamp': timestamp_str,
            'data': data
        }

        # 添加到日志列表
        self.logs.append(log_entry)

    def flush(self):
        """将日志写入文件"""
        with open(self.log_path, 'w') as f:
            json.dump(self.logs, f, indent=self.indent)

    def close(self):
        """关闭日志记录器"""
        # 确保所有日志写入文件
        self.flush()