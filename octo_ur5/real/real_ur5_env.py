import gym
import gym.spaces
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

from octo_ur5.real.utilities import RealsenseCamera
from octo_ur5.real.robot_controller import UR5ArmController


class RealUR5(gym.Env):
    """Gym environment wrapping a real UR5 robot arm with gripper and Realsense camera."""

    def __init__(self, robot=UR5ArmController, camera=RealsenseCamera) -> None:
        self.robot = robot()
        self.camera = camera()

        self.ranges = [
            (0.399, 0.667),   # x
            (-0.270, 0.250),  # y
            (0.033, 0.332),   # z
            (-3.14, 3.14),    # roll
            (-3.14, 3.14),    # pitch
            (-np.pi / 2, np.pi / 2),  # yaw
            (0, 1)            # gripper [0, 1000]
        ]

        self.cup_picked = False
        self.cup_reset = False
        self.mug_picked = False
        self.mug_reset = False

        self.discount = np.float32(1.0)
        self.language_instruction = "Pick up the cup and the mug, and then put them down"

        self.info = dict(
            cup_picked=self.cup_picked, cup_reset=self.cup_reset,
            mug_picked=self.mug_picked, mug_reset=self.mug_reset,
            discount=self.discount, language_instruction=self.language_instruction,
        )
        self._im_size = 256

        self.observation_space = gym.spaces.Dict({
            "image_primary": gym.spaces.Box(low=0, high=255, shape=(256, 256, 3), dtype=np.uint8),
            "proprio": gym.spaces.Box(low=-3.14, high=3.14, shape=(7,), dtype=np.float32),
        })
        self.action_space = gym.spaces.Box(low=-3.14, high=3.14, shape=(7,), dtype=np.float32)

    def get_task(self):
        return {"language_instruction": [self.language_instruction]}

    def read_control_parameter(self, action_rotvec, cup_picked, cup_reset, mug_picked, mug_reset):
        """Update task state and convert rotvec action to euler angles."""
        self.cup_picked = cup_picked
        self.cup_reset = cup_reset
        self.mug_picked = mug_picked
        self.mug_reset = mug_reset
        self.info = dict(
            cup_picked=self.cup_picked, cup_reset=self.cup_reset,
            mug_picked=self.mug_picked, mug_reset=self.mug_reset,
            discount=self.discount, language_instruction=self.language_instruction,
        )

        x, y, z, rx, ry, rz = action_rotvec[:6]
        gripper_position = action_rotvec[6]
        euler_angles = R.from_rotvec([rx, ry, rz]).as_euler('xyz', degrees=False)
        return [x, y, z] + list(euler_angles) + [gripper_position]

    def step(self, action):
        self.robot.step(action)
        reward = self.update_reward()
        terminated = reward == 4
        return self.get_observation(), reward, terminated, False, self.info

    def update_reward(self):
        reward = 0
        if self.cup_picked:
            reward += 1
        if self.cup_picked and self.cup_reset:
            reward += 1
        if self.mug_picked:
            reward += 1
        if self.mug_picked and self.mug_reset:
            reward += 1
        return np.float32(reward)

    def get_observation(self):
        obs = {}
        if isinstance(self.camera, RealsenseCamera):
            rgb, _ = self.camera.get_frames()
            rgb = cv2.resize(rgb, (self._im_size, self._im_size), interpolation=cv2.INTER_AREA)
            obs['image_primary'] = rgb.astype(np.uint8)
        else:
            assert self.camera is None

        state = self.robot.get_current_state()
        obs['proprio'] = state["tcp_position"] + state["tcp_orientation"] + (state["gripper_position"],)
        return obs

    def reset(self):
        self.robot.reset()
        self.cup_picked = False
        self.cup_reset = False
        self.mug_picked = False
        self.mug_reset = False
        self.discount = np.float32(1.0)
        return self.get_observation(), self.info
