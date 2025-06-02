import numpy as np
import cv2
import os
import yaml


# class CameraCalibration:
#     """摄像头校准类，处理相机内参和畸变校正"""
#     def __init__(self):
#         self.left_camera_matrix = None
#         self.right_camera_matrix = None
#         self.left_dist_coeffs = None
#         self.right_dist_coeffs = None
#
#         self.R = None
#         self.T = None
#
#         # No used
#         self.new_camera_matrix = None
#         self.roi = None
#         self.calibrated = False
#
#     def load_calibration(self, calibration_file):
#         """从YAML文件加载校准参数"""
#         try:
#             if not os.path.exists(calibration_file):
#                 print(f"校准文件不存在: {calibration_file}")
#                 return False
#
#             with open(calibration_file, 'r') as f:
#                 calib_data = yaml.safe_load(f)
#
#             self.camera_matrix = np.array(calib_data['camera_matrix'])
#             self.dist_coeffs = np.array(calib_data['dist_coeffs'])
#             self.calibrated = True
#
#             print(f"已加载校准参数，来自: {calibration_file}")
#             return True
#
#         except Exception as e:
#             print(f"加载校准文件时出错: {str(e)}")
#             return False
#
#     def calibrate_camera(self, images, pattern_size=(9, 6), square_size=0.025):
#         """执行相机校准
#
#         Args:
#             images: 校准图像列表
#             pattern_size: 棋盘格内角点数量
#             square_size: 棋盘格方块尺寸(米)
#         """
#         # 准备对象点 (0,0,0), (1,0,0), (2,0,0) ...
#         objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
#         objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * square_size
#
#         # 对象点和图像点列表
#         objpoints = []  # 3D点在现实世界中的位置
#         imgpoints = []  # 2D点在图像平面中的位置
#
#         gray = None
#
#         # 检测棋盘格角点
#         for img in images:
#             gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#
#             # 查找棋盘格角点
#             ret, corners = cv2.findChessboardCorners(
#                 gray, pattern_size, cv2.CALIB_CB_ADAPTIVE_THRESH +
#                                     cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
#             )
#
#             # 如果找到，添加对象点和图像点
#             if ret:
#                 objpoints.append(objp)
#
#                 # 亚像素级角点检测
#                 criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
#                 corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
#                 imgpoints.append(corners2)
#
#         if not objpoints:
#             print("未找到足够的棋盘格角点进行校准")
#             return False
#
#         # 执行校准
#         ret, self.camera_matrix, self.dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
#             objpoints, imgpoints, gray.shape[::-1], None, None
#         )
#
#         if ret:
#             # 计算无畸变的新相机矩阵
#             h, w = gray.shape
#             self.new_camera_matrix, self.roi = cv2.getOptimalNewCameraMatrix(
#                 self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h)
#             )
#             self.calibrated = True
#             print("相机校准成功")
#             return True
#         else:
#             print("相机校准失败")
#             return False
#
#     def undistort_image(self, image):
#         """校正图像畸变"""
#         if not self.calibrated:
#             return image
#
#         # 校正畸变
#         undistorted = cv2.undistort(
#             image,
#             self.camera_matrix,
#             self.dist_coeffs,
#             None,
#             self.new_camera_matrix
#         )
#
#         # 裁剪图像（可选）
#         if self.roi is not None:
#             x, y, w, h = self.roi
#             undistorted = undistorted[y:y + h, x:x + w]
#
#         return undistorted
#
#     def save_calibration(self, filename):
#         """保存校准参数到YAML文件"""
#         if not self.calibrated:
#             print("未校准，无法保存参数")
#             return False
#
#         calib_data = {
#             'camera_matrix': self.camera_matrix.tolist(),
#             'dist_coeffs': self.dist_coeffs.tolist(),
#             'new_camera_matrix': self.new_camera_matrix.tolist() if self.new_camera_matrix is not None else None,
#             'roi': self.roi if self.roi is not None else None
#         }
#
#         try:
#             with open(filename, 'w') as f:
#                 yaml.dump(calib_data, f)
#             print(f"校准参数已保存到: {filename}")
#             return True
#         except Exception as e:
#             print(f"保存校准参数时出错: {str(e)}")
#             return False

class CameraCalibration:
    """相机标定参数管理类"""

    def __init__(self, left_camera_matrix=None, right_camera_matrix=None,
                 left_dist_coeffs=None, right_dist_coeffs=None,
                 rotation_matrix=None, translation_vector=None):
        """
        初始化相机标定参数

        参数:
            left_camera_matrix: 左相机内参矩阵
            right_camera_matrix: 右相机内参矩阵
            left_dist_coeffs: 左相机畸变系数
            right_dist_coeffs: 右相机畸变系数
            rotation_matrix: 旋转矩阵
            translation_vector: 平移向量
        """
        self.left_camera_matrix = left_camera_matrix
        self.right_camera_matrix = right_camera_matrix
        self.left_dist_coeffs = left_dist_coeffs
        self.right_dist_coeffs = right_dist_coeffs
        self.rotation_matrix = rotation_matrix
        self.translation_vector = translation_vector

    @classmethod
    def from_defaults(cls):
        """使用默认参数创建实例"""
        left_camera_matrix = np.array([[1486.1, 0, 576.8997],
                                       [0, 1476.1, 345.9153],
                                       [0, 0, 1]])
        right_camera_matrix = np.array([[1480.6, 0, 603.4164],
                                        [0, 1471.3, 319.7381],
                                        [0, 0, 1]])

        # k1 k2 p1 p2 k3 格式
        left_dist_coeffs = np.array([-0.4820, 0.3408, 0.0077, -0.0042, -0.2191])
        right_dist_coeffs = np.array([-0.4638, -0.0451, 0.0082, -0.0011, 2.3658])

        R = np.array([[1.0000, 0.0005, 0.0052],
                      [-0.00058766, 1.000, 0.0075],
                      [-0.0052, -0.0075, 1.0000]])

        T = np.array([[-59.5834], [0.0843], [-0.0506]])

        return cls(left_camera_matrix, right_camera_matrix,
                   left_dist_coeffs, right_dist_coeffs, R, T)

    def save_to_yaml(self, file_path):
        """
        将相机参数保存到YAML文件

        参数:
            file_path: YAML文件路径
        """
        data = {
            'left_camera_matrix': self.left_camera_matrix.tolist(),
            'right_camera_matrix': self.right_camera_matrix.tolist(),
            'left_dist_coeffs': self.left_dist_coeffs.tolist(),
            'right_dist_coeffs': self.right_dist_coeffs.tolist(),
            'rotation_matrix': self.rotation_matrix.tolist(),
            'translation_vector': self.translation_vector.tolist()
        }

        with open(file_path, 'w') as f:
            yaml.dump(data, f, default_flow_style=None)

    @classmethod
    def load_from_yaml(cls, file_path):
        """
        从YAML文件加载相机参数

        参数:
            file_path: YAML文件路径

        返回:
            CameraCalibration实例
        """
        with open(file_path, 'r') as f:
            data = yaml.safe_load(f)

        return cls(
            np.array(data['left_camera_matrix']),
            np.array(data['right_camera_matrix']),
            np.array(data['left_dist_coeffs']),
            np.array(data['right_dist_coeffs']),
            np.array(data['rotation_matrix']),
            np.array(data['translation_vector'])
        )

    def __str__(self):
        """返回参数的字符串表示"""
        return (f"CameraCalibration(\n"
                f"  left_camera_matrix=\n{self.left_camera_matrix}\n"
                f"  right_camera_matrix=\n{self.right_camera_matrix}\n"
                f"  left_dist_coeffs={self.left_dist_coeffs}\n"
                f"  right_dist_coeffs={self.right_dist_coeffs}\n"
                f"  rotation_matrix=\n{self.rotation_matrix}\n"
                f"  translation_vector=\n{self.translation_vector}\n)")


# 使用示例
if __name__ == "__main__":
    # 创建带有默认参数的实例
    calibration = CameraCalibration.from_defaults()

    # 保存到YAML文件
    calibration.save_to_yaml("camera_params.yaml")
    print("参数已保存到 camera_params.yaml")

    # 从YAML文件加载
    loaded_calibration = CameraCalibration.load_from_yaml("camera_params.yaml")
    print("\n从YAML文件加载的参数:")
    print(loaded_calibration)
    print("loaded_calibration.left_camera_matrix = ",loaded_calibration.left_camera_matrix)