import cv2
from octo_ur5.real.utilities import RealsenseCamera


def test_realsense_camera():
    camera = RealsenseCamera()
    try:
        while True:
            color_image, _ = camera.get_frames()
            color_image = cv2.resize(color_image, (256, 256), interpolation=cv2.INTER_AREA)
            cv2.imshow('Color Image', color_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        camera.release()


if __name__ == "__main__":
    test_realsense_camera()
