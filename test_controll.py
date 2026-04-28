import cv2
import pygame
import threading
import rtde_control
import rtde_receive
# from gripper_controller import GripperController
from gripper_controller import GripperController
import time
import numpy as np

from real_ur5_env.utilities import GamepadController  # 请确保正确导入你定义的 GamepadController 类

# 全局变量，用于控制线程退出
exit_event = threading.Event()

# 初始化 RTDE 控制接口
robot = rtde_control.RTDEControlInterface("192.168.4.100")
info = rtde_receive.RTDEReceiveInterface("192.168.4.100")

# 初始化夹爪控制器
gripper = GripperController('COM5', 115200)
GRIPPER_RANGE = [800, 10]  # [OPEN, CLOSE]
gripper.set_gripper_position(GRIPPER_RANGE[0])  # Initial open

# 安全空间范围（根据你的环境设置）
SAFE_SPACE = {
    'x': (0.399, 0.667),  # 安全的x坐标范围
    'y': (-0.270, 0.250),  # 安全的y坐标范围
    'z': (0.033, 0.332),   # 安全的z坐标范围
}

def is_target_position_safe(target_position):
    x, y, z = target_position
    # TODO: debug  禁用安全检查
    # return (SAFE_SPACE['x'][0] <= x <= SAFE_SPACE['x'][1] and
    #         SAFE_SPACE['y'][0] <= y <= SAFE_SPACE['y'][1] and
    #         SAFE_SPACE['z'][0] <= z <= SAFE_SPACE['z'][1])
    return True

class ArmController:
    def __init__(self, gamepad_controller):
        self.gamepad_controller = gamepad_controller

    def run_arm(self):
        while not exit_event.is_set():
            # 获取当前的手柄状态
            state = self.gamepad_controller.read()

            # 使用手柄状态映射到控制参数
            DR_Target_TCP = self.gamepad_controller.map_gamepad_to_action(state)

            # 检查目标位置是否在安全范围内
            # TODO: debug  禁用安全检查
            if is_target_position_safe(DR_Target_TCP[:3]):
                robot.servoL(np.array(DR_Target_TCP[:6]), velocity, acceleration, dt, lookahead_time, gain)

            time.sleep(0.014)

    def run_gripper(self):
        while not exit_event.is_set():
            # 获取当前的手柄状态
            state = self.gamepad_controller.read()

            # 使用手柄状态映射到控制参数
            DR_Target_TCP = self.gamepad_controller.map_gamepad_to_action(state)

            # 设置夹爪位置
            gripper.set_gripper_position(DR_Target_TCP[6])

            time.sleep(0.1)  # 控制夹爪操作的频率

def main():
    # 初始化手柄控制类
    gamepad_controller = GamepadController()

    # 初始化机械臂控制器并传入手柄控制类
    controller = ArmController(gamepad_controller)

    # 启动手柄控制的后台线程
    gamepad_controller.start()

    # 启动机械臂和夹爪控制线程
    arm_thread = threading.Thread(target=controller.run_arm)
    gripper_thread = threading.Thread(target=controller.run_gripper)

    arm_thread.start()
    gripper_thread.start()

    arm_thread.join()
    gripper_thread.join()

    # 停止手柄控制的后台线程
    gamepad_controller.stop()

if __name__ == "__main__":
    # 控制参数配置
    velocity = 0.1
    acceleration = 0.1
    dt = 1.0 / 25
    lookahead_time = 0.1
    gain = 100

    main()
