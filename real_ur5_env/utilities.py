import pyrealsense2 as rs
import numpy as np
import cv2


class RealsenseCamera:
    def __init__(self):
        # 初始化Realsense相机
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(self.config)

        # 获取内参
        profile = self.pipeline.get_active_profile()
        self.intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        print(f"Camera intrinsic parameters: \nFocal Length (fx, fy): ({self.intrinsics.fx}, {self.intrinsics.fy}) \n"
              f"Principal Point (ppx, ppy): ({self.intrinsics.ppx}, {self.intrinsics.ppy}) \n"
              f"Distortion Coefficients: {self.intrinsics.coeffs}")

    def get_frames(self):
        # 获取彩色和深度图像
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        # 转换为numpy数组
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        return color_image, depth_image

    def release(self):
        # 释放资源
        self.pipeline.stop()
        cv2.destroyAllWindows()


import pygame
import threading

from scipy.spatial.transform import Rotation as R

class GamepadController:
    def __init__(self, Ordinary_speed=0.01, Ordinary_angular_speed=0.1):
        # 初始化Pygame和游戏手柄
        pygame.init()
        pygame.joystick.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        # 初始化状态变量
        self.state = {
            "cross": (0, 0),
            "buttons": [0] * self.joystick.get_numbuttons(),  # 动态获取手柄上的按钮数量
            "axes": [0.0] * self.joystick.get_numaxes()  # 动态获取手柄上的轴数量
        }

        self.Ordinary_speed = Ordinary_speed
        self.Ordinary_angular_speed = Ordinary_angular_speed

        # 线程控制变量
        self.exit_event = threading.Event()
        self.thread = None

        # 任务状态
        self.task_status = {
            "cup_picked": False,
            "cup_reset": False,
            "mug_picked": False,
            "mug_reset": False
        }

        # 标记控制流程的状态
        self.control_active = False

    def read(self):
        """返回当前的手柄状态。"""
        return self.state

    def update(self):
        """更新手柄的状态。"""
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                axis = event.axis
                value = event.value
                print("Axis: ", axis, " Value: ", value)
                self.state["axes"][axis] = value
            elif event.type == pygame.JOYBUTTONDOWN:
                button = event.button
                print("Button: ", button, " down")
                self.state["buttons"][button] = 1
            elif event.type == pygame.JOYBUTTONUP:
                button = event.button
                self.state["buttons"][button] = 0
            elif event.type == pygame.JOYHATMOTION:
                self.state["cross"] = event.value


    def handle_button_down(self, state):
        """根据按下的按钮更新任务状态或控制流程"""

        if state["buttons"][4] == 1:  # 假设按钮2对应第一个任务完成
            self.task_status["cup_picked"] = True
        if state["buttons"][5] == 1 and self.task_status["cup_picked"]:  # 假设按钮3对应第二个任务完成
            self.task_status["cup_reset"] = True
        if state["buttons"][6] == 1:  # 假设按钮4对应第三个任务完成
            self.task_status["mug_picked"] = True
        if state["buttons"][7] == 1 and self.task_status["mug_picked"]:  # 假设按钮5对应第四个任务完成
            self.task_status["mug_reset"] = True

    def run(self):
        """在后台线程中持续更新手柄状态。"""
        while not self.exit_event.is_set():
            self.update()
            pygame.time.wait(10)  # 等待10毫秒以避免占用过多CPU

    def start(self):
        """启动手柄状态更新的后台线程。"""
        if not self.thread:
            self.thread = threading.Thread(target=self.run)
            self.thread.start()

    def stop(self):
        """停止手柄状态更新的后台线程。"""
        self.exit_event.set()
        if self.thread:
            self.thread.join()
            self.thread = None

    def reset_tasks(self):
        """重置任务状态"""
        for task in self.task_status:
            self.task_status[task] = False

    def map_gamepad_to_action(self, state, current_action_rotvec, current_gripper_position):
        """
        将游戏手柄的输入映射到机械臂和夹爪的控制参数上。

        :param state: 游戏手柄的状态字典
        :return: action 列表 [x, y, z, r, p, y, gripper_position]
        """
        DR_Target_TCP = current_action_rotvec


        # 处理机械臂的平移控制（假设 axes[0], axes[1], axes[2] 控制机械臂的 XYZ）
        # if state["buttons"][6] == 0:

        if state["buttons"][11] == 0 and state["buttons"][10] == 0:
            if abs(state["axes"][0]) > 0.01:         # 假设0.01为死区阈值
                DR_Target_TCP[1] += -1 * state["axes"][0] * self.Ordinary_speed
            if abs(state["axes"][1]) > 0.01:
                DR_Target_TCP[0] += -1 * state["axes"][1] * self.Ordinary_speed
            if abs(state["axes"][3]) > 0.01:
                DR_Target_TCP[2] += -1 * state["axes"][3] * self.Ordinary_speed


        # 处理机械臂的旋转控制（假设 axis[2] 和 buttons 控制机械臂的旋转）
        if state["buttons"][11] == 0:
            # 使用 axis[2] 控制绕 x 轴的旋转
            rotation = R.from_rotvec(state["axes"][2] * self.Ordinary_angular_speed * np.array([1.0, 0.0, 0.0]))
            current_q = R.from_rotvec(DR_Target_TCP[3:6])
            DR_Target_TCP[3:6] = (rotation * current_q).as_rotvec()

        # 使用 button0 和 button3 控制绕 y 轴的旋转
        if state["buttons"][0] != 0:  # 假设 button0 对应正方向旋转
            rotation = R.from_rotvec(self.Ordinary_angular_speed * np.array([0.0, 1.0, 0.0]))
            current_q = R.from_rotvec(DR_Target_TCP[3:6])
            DR_Target_TCP[3:6] = (rotation * current_q).as_rotvec()

        elif state["buttons"][3] != 0:  # 假设 button3 对应负方向旋转
            rotation = R.from_rotvec(-self.Ordinary_angular_speed * np.array([0.0, 1.0, 0.0]))
            current_q = R.from_rotvec(DR_Target_TCP[3:6])
            DR_Target_TCP[3:6] = (rotation * current_q).as_rotvec()

        # 使用 button8 和 button9 控制绕 z 轴的旋转
        if state["buttons"][8] != 0:  # 假设 button8 对应正方向旋转
            rotation = R.from_rotvec(self.Ordinary_angular_speed * np.array([0.0, 0.0, 1.0]))
            current_q = R.from_rotvec(DR_Target_TCP[3:6])
            DR_Target_TCP[3:6] = (rotation * current_q).as_rotvec()

        elif state["buttons"][9] != 0:  # 假设 button9 对应负方向旋转
            rotation = R.from_rotvec(-self.Ordinary_angular_speed * np.array([0.0, 0.0, 1.0]))
            current_q = R.from_rotvec(DR_Target_TCP[3:6])
            DR_Target_TCP[3:6] = (rotation * current_q).as_rotvec()

        # 处理夹爪的开合（假设 buttons[2] 控制夹爪开，buttons[1] 控制夹爪关）
        # gripper_state = 0 sdlfiasnfawn gf 这不是纯傻逼吗，每次都初始化了怎么玩？大改！整个就传整个动作进来，包括gripperpos！

        # gripper_state = current_gripper_position
        # if state["buttons"][2] == 1:
        #     gripper_state = min(gripper_state + 0.1, 1.0)
        #     gripper_state
        # if state["buttons"][1] == 1:
        #     gripper_state = max(gripper_state - 0.1, 0.0)
        #
        # # 假设夹爪的开合范围为 [10, 800]
        # gripper_position = int(800 - gripper_state * (800 - 10))
        gripper_position = current_gripper_position
        if state["buttons"][2] == 1:
            gripper_position = min(gripper_position + 10, 800)     # 800 - 10 is the bound    + - 10 is the temp
        if state["buttons"][1] == 1:
            gripper_position = max(gripper_position - 10, 10)

        # 返回最终的 action 参数列表
        return DR_Target_TCP + [gripper_position]


