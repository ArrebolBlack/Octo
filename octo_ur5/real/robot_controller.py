import numpy as np
import time
import rtde_control
import rtde_receive
from gripper_controller import GripperController

from scipy.spatial.transform import Rotation as R

class UR5ArmController:
    def __init__(self, robot_ip="192.168.4.100", gripper_port="COM5", gripper_baudrate=115200):
        # 初始化UR5机械臂的控制和接收接口
        self.robot = rtde_control.RTDEControlInterface(robot_ip)
        self.info = rtde_receive.RTDEReceiveInterface(robot_ip)

        # 初始化夹爪控制
        self.gripper = GripperController(gripper_port, gripper_baudrate)
        self.gripper_open_pos = 800  # 全开位置
        self.gripper_close_pos = 10  # 全关位置
        self.gripper.set_gripper_position(self.gripper_open_pos)  # 初始化全开

        # 默认速度和加速度
        self.velocity = 0.1
        self.acceleration = 0.1
        self.dt = 1.0 / 25
        self.lookahead_time = 0.1
        self.gain = 100

        self.initial_position = np.array([0.507, -0.091, 0.173])  # 初始位置
        self.initial_orientation = np.array([1.100, 1.355, 1.129])  # 初始姿态

    def reset(self):
        """
        重置机械臂到初始位置和姿态。
        """
        self.robot.moveL(np.concatenate((self.initial_position, self.initial_orientation)), 0.5, 0.3)
        self.gripper.set_gripper_position(self.gripper_open_pos)
        print("Arm reset to initial position.")
        print("Gripper reset to initial position.")

    def step(self, action):
        """
        执行一步动作，包括机械臂移动和夹爪操作。传入的是action
        """
        target_tcp_xyz = action[0:3]
        target_tcp_ori_euler = action[3:6]

        # 将欧拉角转换为旋转矢量。
        rotation = R.from_euler('xyz', target_tcp_ori_euler, degrees=False)
        target_tcp_ori = rotation.as_rotvec()

        # 组合位置和姿态
        target_tcp_position = np.concatenate((target_tcp_xyz, target_tcp_ori))

        # 获取夹爪位置，并检查其是否在范围内
        gripper_position = int(action[-1])  # 将夹爪位置转换为整数
        if not (self.gripper_open_pos >= gripper_position >= self.gripper_close_pos):
            print(f"Warning: Gripper position {gripper_position} out of range. Clipping to valid range.")
            gripper_position = np.clip(gripper_position, self.gripper_close_pos, self.gripper_open_pos)

        # 机械臂移动
        if target_tcp_position is not None:
            # TODO: debug for safe range is too small.
            # safe_position = self.check_and_correct_position(target_tcp_position)
            safe_position = target_tcp_position
            self.robot.servoL(np.array(safe_position), self.velocity, self.acceleration, self.dt, self.lookahead_time, self.gain)
        else:
            print("target_tcp_position is None!")
# 可能需要分别停顿，以免夹爪运动影响机械臂的感知运动。。等实机测试了根据结果再考虑
        # 夹爪动作
        if gripper_position is not None :
            self.gripper.set_gripper_position(gripper_position)
        else:
            print("gripper action is None!!")

        time.sleep(0.014)  # 适当延时，模拟控制周期

    def check_and_correct_position(self, target_position):
        """
        检查目标位置是否在安全范围内，如果超出则修正并打印警告。
        """
        safe_position = target_position.copy()

        if not (0.399 <= target_position[0] <= 0.667):
            print(f"Warning: X position {target_position[0]} out of safe range. Clipping to range [0.399, 0.667].")
        safe_position[0] = np.clip(target_position[0], 0.399, 0.667)  # x轴

        if not (-0.270 <= target_position[1] <= 0.250):
            print(f"Warning: Y position {target_position[1]} out of safe range. Clipping to range [-0.270, 0.250].")
        safe_position[1] = np.clip(target_position[1], -0.270, 0.250)  # y轴

        if not (0.033 <= target_position[2] <= 0.332):
            print(f"Warning: Z position {target_position[2]} out of safe range. Clipping to range [0.033, 0.332].")
        safe_position[2] = np.clip(target_position[2], 0.033, 0.332)  # z轴

        return safe_position

    def get_current_state(self):
        """
        获取当前的机械臂状态，包括TCP位置、速度、力等。
        """
        tcp_pose = self.info.getActualTCPPose() # return x, y, z, rx, ry, rz

        # 将旋转向量转换为欧拉角（r, p, y)
        rotation = R.from_rotvec(tcp_pose[3:6])
        euler_angles = rotation.as_euler('xyz', degrees=False)
        state = {
            "tcp_position": tcp_pose[:3],
            "tcp_orientation": euler_angles,
            # "tcp_velocity": self.info.getActualTCPSpeed(),
            # "tcp_force": self.info.getActualTCPForce(),
            "gripper_position": self.gripper.last_send_value
        }
        return state

    def stop(self):
        """
        停止机械臂和夹爪的控制。
        """
        self.robot.servoStop()
        self.gripper.set_gripper_position(self.gripper_open_pos)
        print("Arm and gripper stopped.")



