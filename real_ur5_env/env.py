
import time
import math
import random

import gym
import gym.spaces
import numpy as np

from utilities import RealsenseCamera
from robot import UR5ArmController
from collections import namedtuple
# from attrdict import AttrDict
# from tqdm import tqdm


import cv2

import logging
from scipy.spatial.transform import Rotation as R


class RealUR5(gym.Env):

    SIMULATION_STEP_DELAY = 1 / 240.

    def __init__(self, robot=UR5ArmController, camera=RealsenseCamera) -> None:
        self.robot = robot()
        self.camera = camera()

        self.ranges = [
            (0.399, 0.667),  # 安全的x坐标范围
            (-0.270, 0.250),  # 安全的y坐标范围
            (0.033, 0.332),  # 安全的z坐标范围
            (-3.14, 3.14),  # roll
            (-3.14, 3.14),  # pitch
            (-np.pi / 2, np.pi / 2),  # yaw
            (0, 1)  # gripper opening length [0, 1000]
        ]

        # For calculating the reward
        self.cup_picked = False
        self.cup_reset = False
        self.mug_picked = False
        self.mug_reset = False

        self.discount = np.float32(1.0)
        self.language_instruction = "Pick up the cup and the mug, and then put them down"

        self.info = dict(cup_picked=self.cup_picked, cup_reset=self.cup_reset, mug_picked=self.mug_picked,
                         mug_reset=self.mug_reset, discount=self.discount,
                         language_instruction=self.language_instruction)
        im_size = 256
        self._im_size = im_size

        """
        observation: primary img 256 256 3
        action dim=7 x y z r p y g

        """

        self.observation_space = gym.spaces.Dict(
            {
                "image_primary": gym.spaces.Box(
                    low=0,  # 对应 specs.BoundedArray 的 minimum=0
                    high=255,  # 对应 specs.BoundedArray 的 maximum=255
                    shape=(256, 256, 3),  # 对应 specs.BoundedArray 的 shape=(256, 256, 3)
                    dtype=np.uint8  # 对应 specs.BoundedArray 的 dtype=np.uint8
                ),
                "proprio": gym.spaces.Box(
                    low=np.ones((7,)) * -3.14,  # 对应 specs.BoundedArray 的 minimum=-3.14
                    high=np.ones((7,)) * 3.14,  # 对应 specs.BoundedArray 的 maximum=3.14
                    dtype=np.float32  # 对应 specs.BoundedArray 的 dtype=np.float32
                ),
            }
        )

        self.action_space = gym.spaces.Box(
            low=np.ones((7,)) * -3.14,  # 对应 specs.BoundedArray 的 minimum=-3.14
            high=np.ones((7,)) * 3.14,  # 对应 specs.BoundedArray 的 maximum=3.14
            dtype=np.float32  # 对应 specs.BoundedArray 的 dtype=np.float32
        )

    def get_task(self):
        return {
            "language_instruction": [self.language_instruction],
        }

    def read_control_parameter(self, action_rotvec, cup_picked, cup_reset, mug_picked, mug_reset):
        """
        接收外部控制参数，并更新环境的任务状态。
        接收旋转矢量，返回欧拉角

        :param action: 机械臂和夹爪的控制参数 (例如 [x, y, z, r, p, y, gripper_position])
        :param cup_picked: 表示杯子是否被拾起的布尔值
        :param cup_reset: 表示杯子是否需要重置的布尔值
        :param mug_picked: 表示杯子是否被拾起的布尔值
        :param mug_reset: 表示杯子是否需要重置的布尔值
        :return: 返回处理后的 action 参数  欧拉角
        """
        # 更新任务状态
        self.cup_picked = cup_picked
        self.cup_reset = cup_reset
        self.mug_picked = mug_picked
        self.mug_reset = mug_reset
        self.info = dict(cup_picked=self.cup_picked, cup_reset=self.cup_reset, mug_picked=self.mug_picked,
                         mug_reset=self.mug_reset, discount=self.discount,
                         language_instruction=self.language_instruction)

        # 处理： rotvec -> euler
        # 返回 action 参数，用于进一步的机械臂和夹爪控制
        # 提取位置和旋转向量
        x, y, z, rx, ry, rz = action_rotvec[:6]
        gripper_position = action_rotvec[6]
        # 将旋转向量转换为欧拉角
        rotation_vector = [rx, ry, rz]
        rotation = R.from_rotvec(rotation_vector)
        euler_angles = rotation.as_euler('xyz', degrees=False)  # 返回 [roll, pitch, yaw]

        # 组合新的 action 参数列表
        action_euler = [x, y, z] + list(euler_angles)

        action_euler.append(gripper_position)

        # 返回处理后的 action 参数
        return action_euler

    def step(self, action):
        """
        action: (x, y, z, roll, pitch, yaw, gripper_opening_length) for End Effector Position Control
                (a1, a2, a3, a4, a5, a6, a7, gripper_opening_length) for Joint Position Control
        control_method:  'end' for end effector position control
                         'joint' for joint position control
        """

        self.robot.step(action)

        reward = self.update_reward()

        terminated = True if reward == 4 else False

        truncated = False

        info = self.info

        return self.get_observation(), reward, terminated, truncated, info


    def update_reward(self):
        reward = 0

        if self.cup_picked:
            print('Cup picked!')
            reward += 1
        if self.cup_picked:
            if self.cup_reset:
                print('Cup reset!')
                reward += 1

        if self.mug_picked:
            print('Mug picked!')
            reward += 1
        if self.mug_picked:
            if self.mug_reset:
                print('Mug reset!')
                reward += 1

        return np.float32(reward)

    def get_observation(self):

        obs = {}
        if isinstance(self.camera, RealsenseCamera):
            rgb, depth = self.camera.get_frames()

            # if rgb.shape[:2] != [self._im_size, self._im_size]:
            rgb = cv2.resize(rgb, (self._im_size, self._im_size), interpolation=cv2.INTER_AREA)

            obs['image_primary'] = rgb.astype(np.uint8)

        else:
            assert self.camera is None

        state = self.robot.get_current_state()

        obs['proprio'] = state["tcp_position"] + state["tcp_orientation"] + (state["gripper_position"],)
        # print(obs['proprio'])

        return obs


    def reset(self):
        print("Starting Reset...")
        self.robot.reset()

        self.cup_picked = False
        self.cup_reset = False
        self.mug_picked = False
        self.mug_reset = False
        self.discount = np.float32(1.0)
        # self.info = dict(cup_picked=self.cup_picked, cup_reset=self.cup_reset, mug_picked=self.mug_picked, mug_reset=self.mug_reset, discount=self.discount)
        info = self.info
        print("Finishing reset...")
        # return obs, info
        return self.get_observation(), info

    # def render(self):
    #     None

#    def close(self):


