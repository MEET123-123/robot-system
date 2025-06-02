from .logger import Logger


class CombinedLogger(Logger):
    """组合日志记录器 - 同时使用多个日志记录器"""

    def __init__(self, loggers):
        """
        初始化组合日志记录器

        参数:
            loggers: 日志记录器列表
        """
        super().__init__()
        self.loggers = loggers

    def log(self, data, timestamp=None):
        """
        使用所有日志记录器记录数据

        参数:
            data: 要记录的数据
            timestamp: 时间戳(可选)
        """
        for logger in self.loggers:
            logger.log(data, timestamp)

    def close(self):
        """关闭所有日志记录器"""
        for logger in self.loggers:
            logger.close()