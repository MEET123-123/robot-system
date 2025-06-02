import numpy as np
import cv2
import os
import yaml


class CameraCalibration:
    """摄像头校准类，处理相机内参和畸变校正"""

    def __init__(self):
        self.camera_matrix = None
        self.dist_coeffs = None
        self.new_camera_matrix = None
        self.roi = None
        self.calibrated = False

    def load_calibration(self, calibration_file):
        """从YAML文件加载校准参数"""
        try:
            if not os.path.exists(calibration_file):
                print(f"校准文件不存在: {calibration_file}")
                return False

            with open(calibration_file, 'r') as f:
                calib_data = yaml.safe_load(f)

            self.camera_matrix = np.array(calib_data['camera_matrix'])
            self.dist_coeffs = np.array(calib_data['dist_coeffs'])
            self.calibrated = True

            print(f"已加载校准参数，来自: {calibration_file}")
            return True

        except Exception as e:
            print(f"加载校准文件时出错: {str(e)}")
            return False

    def calibrate_camera(self, images, pattern_size=(9, 6), square_size=0.025):
        """执行相机校准

        Args:
            images: 校准图像列表
            pattern_size: 棋盘格内角点数量
            square_size: 棋盘格方块尺寸(米)
        """
        # 准备对象点 (0,0,0), (1,0,0), (2,0,0) ...
        objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * square_size

        # 对象点和图像点列表
        objpoints = []  # 3D点在现实世界中的位置
        imgpoints = []  # 2D点在图像平面中的位置

        gray = None

        # 检测棋盘格角点
        for img in images:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # 查找棋盘格角点
            ret, corners = cv2.findChessboardCorners(
                gray, pattern_size, cv2.CALIB_CB_ADAPTIVE_THRESH +
                                    cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
            )

            # 如果找到，添加对象点和图像点
            if ret:
                objpoints.append(objp)

                # 亚像素级角点检测
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                imgpoints.append(corners2)

        if not objpoints:
            print("未找到足够的棋盘格角点进行校准")
            return False

        # 执行校准
        ret, self.camera_matrix, self.dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, gray.shape[::-1], None, None
        )

        if ret:
            # 计算无畸变的新相机矩阵
            h, w = gray.shape
            self.new_camera_matrix, self.roi = cv2.getOptimalNewCameraMatrix(
                self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h)
            )
            self.calibrated = True
            print("相机校准成功")
            return True
        else:
            print("相机校准失败")
            return False

    def undistort_image(self, image):
        """校正图像畸变"""
        if not self.calibrated:
            return image

        # 校正畸变
        undistorted = cv2.undistort(
            image,
            self.camera_matrix,
            self.dist_coeffs,
            None,
            self.new_camera_matrix
        )

        # 裁剪图像（可选）
        if self.roi is not None:
            x, y, w, h = self.roi
            undistorted = undistorted[y:y + h, x:x + w]

        return undistorted

    def save_calibration(self, filename):
        """保存校准参数到YAML文件"""
        if not self.calibrated:
            print("未校准，无法保存参数")
            return False

        calib_data = {
            'camera_matrix': self.camera_matrix.tolist(),
            'dist_coeffs': self.dist_coeffs.tolist(),
            'new_camera_matrix': self.new_camera_matrix.tolist() if self.new_camera_matrix is not None else None,
            'roi': self.roi if self.roi is not None else None
        }

        try:
            with open(filename, 'w') as f:
                yaml.dump(calib_data, f)
            print(f"校准参数已保存到: {filename}")
            return True
        except Exception as e:
            print(f"保存校准参数时出错: {str(e)}")
            return False