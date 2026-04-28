import cv2
import numpy as np
from real_ur5_env.utilities import RealsenseCamera  # 根据你的实际情况调整导入路径

def test_realsense_camera():
    # 初始化Realsense相机
    camera = RealsenseCamera()

    try:
        while True:
            # 获取彩色和深度图像
            color_image, depth_image = camera.get_frames()
            color_image = cv2.resize(color_image, (256, 256), interpolation=cv2.INTER_AREA)

            # 显示彩色图像
            cv2.imshow('Color Image', color_image)

            # # 将深度图像归一化到0-255之间
            # depth_image_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
            # depth_image_normalized = np.uint8(depth_image_normalized)
            #
            # # 显示深度图像
            # cv2.imshow('Depth Image', depth_image_normalized)

            # 按下 'q' 键退出循环
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # 释放资源
        camera.release()

if __name__ == "__main__":
    test_realsense_camera()
