import cv2
import numpy as np
import os
from .camera_calibration import CameraCalibration


class CameraNode:
    """手术机器人摄像头接口"""

    def __init__(self,yaml_path):

        self.yaml_path = yaml_path
        self.loaded_calibration = CameraCalibration.load_from_yaml(self.yaml_path)

        # 初始化参数
        self.camera_id = 0
        self.fps = 60
        self.resolution = (2560,720)
        self.undistort = False
        self.focal = self.loaded_calibration.left_camera_matrix[0][0]
        self.baseline = self.loaded_calibration.translation_vector[0]
        self.cx = self.loaded_calibration.left_camera_matrix[0][2]
        self.cy = self.loaded_calibration.translation_vector[1][2]
        self.img_size = (1280, 720)

        # 初始化摄像头
        self.cap = None
        self.initialize_camera()

        self.save_path = "../../../../tmp"
        self.left_path = os.path.join(self.save_path, "left")
        self.right_path = os.path.join(self.save_path, "right")
        self._create_directory(self.left_path)
        self._create_directory(self.right_path)

        self.frame_count = 0

    def _create_directory(self, path):
        """创建目录（如果不存在）"""
        if not os.path.exists(path):
            try:
                os.makedirs(path, exist_ok=True)
                print(f"创建目录: {path}")
            except OSError as e:
                print(f"无法创建目录 {path}: {e}")
                raise

    def initialize_camera(self):
        """初始化摄像头设备"""
        try:
            # 释放已存在的摄像头资源
            if self.cap is not None:
                self.cap.release()

            # 打开摄像头
            self.cap = cv2.VideoCapture(self.camera_id)

            # 设置分辨率
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])

            # 设置帧率
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)

            # 检查摄像头是否成功打开
            if not self.cap.isOpened():
                print(f"无法打开摄像头 ID: {self.camera_id}")
                return

            print(f"摄像头已初始化 - 分辨率: {self.resolution[0]}x{self.resolution[1]}, 帧率: {self.fps} FPS")

        except Exception as e:
            print(f"摄像头初始化错误: {str(e)}")

    def calibration_undistort_image(self,frame):
        left_frame = frame[0:720, 0:1280]
        right_frame = frame[0:720, 1280:2560]

        left_rectify, right_rectify, left_map, right_map, Q, valid_roi1, valid_roi2 = cv2.stereoRectify(
            self.loaded_calibration.left_camera_matrix,
            self.loaded_calibration.left_dist_coeffs,
            self.loaded_calibration.right_camera_matrix,
            self.loaded_calibration.right_dist_coeffs,
            self.img_size, self.loaded_calibration.rotation_matrix, self.loaded_calibration.translation_vector)
        maplx, maply = cv2.initUndistortRectifyMap(self.loaded_calibration.left_camera_matrix,
                                                   self.loaded_calibration.left_dist_coeffs,
                                                   left_rectify, left_map,
                                                   self.img_size, cv2.CV_16SC2)
        maprx, mapry = cv2.initUndistortRectifyMap(self.loaded_calibration.right_camera_matrix,
                                                   self.loaded_calibration.right_dist_coeffs,
                                                   right_rectify, right_map,
                                                   self.img_size, cv2.CV_16SC2)
        left_image_corrected = cv2.remap(left_frame, maplx, maply, cv2.INTER_LINEAR)
        right_image_corrected = cv2.remap(right_frame, maprx, mapry, cv2.INTER_LINEAR)
        return left_image_corrected, right_image_corrected

    def publish_image(self):
        """捕获并发布图像"""

        # 加入可拍照的选项并保存到tem文件夹下
        if self.cap is None or not self.cap.isOpened():
            return

        try:
            while self.cap.isOpened():
                # 读取一帧
                ret, frame = self.cap.read()
                if not ret:
                    print("无法获取图像帧")
                    return
                left_frame = frame[0:720, 0:1280]
                right_frame = frame[0:720, 1280:2560]
                # 图像去畸变（如果已校准）
                if not self.undistort:
                    left_frame,right_frame = self.calibration_undistort_image(frame)

                cv2.imshow('left', left_frame)
                cv2.imshow('right', right_frame)

                key = cv2.waitKey(1) & 0xFF
                if key == ord(' '):  # 按下空格键保存图像
                    self.frame_count += 1
                    filename_left = f"left_{self.frame_count}.jpg"
                    filename_right = f"right_{self.frame_count}.jpg"
                    cv2.imwrite(os.path.join(self.left_path, filename_left), left_frame)
                    cv2.imwrite(os.path.join(self.right_path, filename_right), right_frame)

                    print(f"图片已保存: {filename_left}")
                    print(f"图片已保存: {filename_right}")

                if key == ord('q'):
                    break

        except Exception as e:
            print(f"处理图像时出错: {str(e)}")

    def destroy_node(self):
        """节点销毁时释放资源"""
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    node = CameraNode()
    try:
        node.publish_image()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()