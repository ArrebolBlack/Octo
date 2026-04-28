import os
import numpy as np
import time
import rtde_control
import rtde_receive
from octo_ur5.real.gripper_controller import GripperController
from scipy.spatial.transform import Rotation as R


class UR5ArmController:
    def __init__(self, robot_ip=None, gripper_port=None, gripper_baudrate=115200):
        robot_ip = robot_ip or os.environ.get("UR5_ROBOT_IP", "192.168.4.100")
        gripper_port = gripper_port or os.environ.get("UR5_GRIPPER_PORT", "COM5")

        self.robot = rtde_control.RTDEControlInterface(robot_ip)
        self.info = rtde_receive.RTDEReceiveInterface(robot_ip)

        self.gripper = GripperController(gripper_port, gripper_baudrate)
        self.gripper_open_pos = 800
        self.gripper_close_pos = 10
        self.gripper.set_gripper_position(self.gripper_open_pos)

        self.velocity = 0.1
        self.acceleration = 0.1
        self.dt = 1.0 / 25
        self.lookahead_time = 0.1
        self.gain = 100

        self.initial_position = np.array([0.507, -0.091, 0.173])
        self.initial_orientation = np.array([1.100, 1.355, 1.129])

    def reset(self):
        self.robot.moveL(np.concatenate((self.initial_position, self.initial_orientation)), 0.5, 0.3)
        self.gripper.set_gripper_position(self.gripper_open_pos)
        print("Arm and gripper reset to initial position.")

    def step(self, action):
        target_tcp_xyz = action[0:3]
        target_tcp_ori_euler = action[3:6]

        rotation = R.from_euler('xyz', target_tcp_ori_euler, degrees=False)
        target_tcp_ori = rotation.as_rotvec()
        target_tcp_position = np.concatenate((target_tcp_xyz, target_tcp_ori))

        gripper_position = int(action[-1])
        if not (self.gripper_close_pos <= gripper_position <= self.gripper_open_pos):
            print(f"Warning: Gripper position {gripper_position} out of range. Clipping.")
            gripper_position = np.clip(gripper_position, self.gripper_close_pos, self.gripper_open_pos)

        self.robot.servoL(np.array(target_tcp_position), self.velocity, self.acceleration,
                          self.dt, self.lookahead_time, self.gain)
        self.gripper.set_gripper_position(gripper_position)
        time.sleep(0.014)

    def check_and_correct_position(self, target_position):
        safe_position = target_position.copy()
        safe_position[0] = np.clip(target_position[0], 0.399, 0.667)
        safe_position[1] = np.clip(target_position[1], -0.270, 0.250)
        safe_position[2] = np.clip(target_position[2], 0.033, 0.332)
        return safe_position

    def get_current_state(self):
        tcp_pose = self.info.getActualTCPPose()
        rotation = R.from_rotvec(tcp_pose[3:6])
        euler_angles = rotation.as_euler('xyz', degrees=False)
        return {
            "tcp_position": tcp_pose[:3],
            "tcp_orientation": euler_angles,
            "gripper_position": self.gripper.last_send_value
        }

    def stop(self):
        self.robot.servoStop()
        self.gripper.set_gripper_position(self.gripper_open_pos)
        print("Arm and gripper stopped.")
