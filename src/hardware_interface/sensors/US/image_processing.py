import cv2
import numpy as np


class UltrasoundImageProcessor:
    """超声图像处理类，提供图像增强和分析功能"""

    def __init__(self):
        # 处理参数
        self.median_kernel_size = 3
        self.equalize_histogram = True
        self.edge_enhancement = False
        self.contrast_factor = 1.0
        self.brightness_factor = 0

    def process(self, image):
        """处理超声图像

        Args:
            image: 输入的超声图像 (OpenCV格式)

        Returns:
            处理后的图像
        """
        if image is None or image.size == 0:
            return None

        # 复制原始图像
        processed = image.copy()

        # 应用中值滤波降噪
        if self.median_kernel_size > 0:
            processed = cv2.medianBlur(processed, self.median_kernel_size)

        # 直方图均衡化增强对比度
        if self.equalize_histogram:
            processed = cv2.equalizeHist(processed)

        # 调整对比度和亮度
        if self.contrast_factor != 1.0 or self.brightness_factor != 0:
            processed = cv2.convertScaleAbs(
                processed,
                alpha=self.contrast_factor,
                beta=self.brightness_factor
            )

        # 边缘增强 (可选)
        if self.edge_enhancement:
            # 使用拉普拉斯算子增强边缘
            laplacian = cv2.Laplacian(processed, cv2.CV_64F)
            processed = cv2.convertScaleAbs(processed - laplacian)

        return processed

    def set_median_kernel_size(self, size):
        """设置中值滤波核大小"""
        self.median_kernel_size = size

    def enable_histogram_equalization(self, enable=True):
        """启用/禁用直方图均衡化"""
        self.equalize_histogram = enable

    def enable_edge_enhancement(self, enable=True):
        """启用/禁用边缘增强"""
        self.edge_enhancement = enable

    def set_contrast(self, factor):
        """设置对比度因子"""
        self.contrast_factor = factor

    def set_brightness(self, factor):
        """设置亮度因子"""
        self.brightness_factor = factor