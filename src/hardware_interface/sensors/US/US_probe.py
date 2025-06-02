import numpy as np
import cv2
import time


class UltrasoundProbe:
    """超声探头抽象类，提供与超声设备的接口"""

    def __init__(self, model="default"):
        self.model = model
        self.connected = False
        self.configured = False
        self.resolution = [640, 480]
        self.frequency = 5.0  # MHz
        self.gain = 50.0  # %
        self.depth = 15.0  # cm

    def connect(self, device_id):
        """连接到超声探头"""
        try:
            # 实际应用中应替换为真实的设备连接代码
            # 例如：self.device = SomeUltrasoundDevice(device_id)

            # 模拟连接过程
            time.sleep(0.5)
            self.connected = True
            print(f"已连接到超声探头 - 型号: {self.model}, ID: {device_id}")
            return True
        except Exception as e:
            print(f"连接超声探头失败: {str(e)}")
            return False

    def disconnect(self):
        """断开与超声探头的连接"""
        if not self.connected:
            return

        # 实际应用中应释放设备资源
        # 例如：self.device.close()

        self.connected = False
        self.configured = False
        print("已断开与超声探头的连接")

    def configure(self, resolution=None, frequency=None, gain=None, depth=None):
        """配置超声探头参数"""
        if not self.connected:
            print("超声探头未连接")
            return False

        try:
            if resolution:
                self.resolution = resolution
            if frequency:
                self.frequency = frequency
            if gain:
                self.gain = gain
            if depth:
                self.depth = depth

            # 实际应用中应发送配置命令到设备
            # 例如：self.device.set_resolution(resolution)

            self.configured = True
            print(f"超声探头配置成功 - 分辨率: {resolution}, 频率: {frequency}MHz, "
                  f"增益: {gain}%, 深度: {depth}cm")
            return True
        except Exception as e:
            print(f"配置超声探头失败: {str(e)}")
            return False

    def capture_image(self):
        """捕获超声图像

        返回:
            (success, image): success为布尔值，表示是否成功获取图像
                              image是OpenCV格式的超声图像
        """
        if not self.connected or not self.configured:
            print("超声探头未连接或未配置")
            return False, None

        try:
            # 实际应用中应从设备获取真实图像
            # 例如：raw_data = self.device.get_image()
            #       image = self._process_raw_data(raw_data)

            # 模拟生成超声图像
            image = self._generate_simulation_image()

            return True, image
        except Exception as e:
            print(f"捕获超声图像失败: {str(e)}")
            return False, None

    def _generate_simulation_image(self):
        """生成模拟超声图像（仅用于演示）"""
        width, height = self.resolution

        # 创建黑色背景
        image = np.zeros((height, width), dtype=np.uint8)

        # 添加一些随机斑点模拟超声回声
        num_spots = np.random.randint(500, 2000)
        for _ in range(num_spots):
            x = np.random.randint(0, width)
            y = np.random.randint(0, height)
            size = np.random.randint(1, 3)
            intensity = np.random.randint(100, 255)
            cv2.circle(image, (x, y), size, intensity, -1)

        # 添加一些模拟的组织结构
        for _ in range(3):
            center_x = np.random.randint(width // 4, width * 3 // 4)
            center_y = np.random.randint(height // 4, height * 3 // 4)
            radius = np.random.randint(height // 8, height // 4)
            intensity = np.random.randint(50, 150)

            # 绘制椭圆
            angle = np.random.randint(0, 180)
            axes = (radius, radius // 2)
            cv2.ellipse(image, (center_x, center_y), axes, angle, 0, 360, intensity, -1)

        # 添加噪声
        noise = np.random.normal(0, 10, image.shape).astype(np.uint8)
        image = cv2.add(image, noise)

        return image