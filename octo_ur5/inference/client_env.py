"""Server-side Gym environment that communicates with a robot via TCP.

Used by eval scripts (eval_real.py) on the GPU machine. The robot machine
runs client_robot_tcp.py. This env acts as the Gym interface that the
Octo policy interacts with, forwarding actions over TCP.

Architecture:
    Octo policy  →  client_env.py (GPU)  ←TCP→  client_robot_tcp.py (robot)
"""

import gym
import numpy as np
import socket
import pickle


class UR5Gym(gym.Env):
    """Gym environment that communicates with a real UR5 robot via TCP."""

    def __init__(self, port):
        self.im_size = 256

        self.observation_space = gym.spaces.Dict({
            "image_primary": gym.spaces.Box(
                low=0, high=255, shape=(256, 256, 3), dtype=np.uint8),
            "proprio": gym.spaces.Box(
                low=-3.14, high=3.14, shape=(7,), dtype=np.float32),
        })
        self.action_space = gym.spaces.Box(
            low=-3.14, high=3.14, shape=(7,), dtype=np.float32)

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(("", port))
        self.server_socket.listen(5)
        print(f"UR5Gym: listening for robot client on port {port}...")

    def _recv_obs(self, client_socket):
        data_length = pickle.loads(client_socket.recv(1024))
        received_data = b""
        while len(received_data) < data_length:
            packet = client_socket.recv(1024 * 33)
            if packet == b"EndTransmission":
                break
            received_data += packet
        return pickle.loads(received_data)

    def step(self, action):
        client_socket, addr = self.server_socket.accept()
        step_message = pickle.dumps({'type': 'step', 'action': action})
        client_socket.sendall(step_message)
        received = self._recv_obs(client_socket)
        obs = received['obs']
        truncated = received['truncated']
        client_socket.close()
        return obs, 0, False, truncated, {}

    def reset(self):
        client_socket, addr = self.server_socket.accept()
        reset_message = pickle.dumps({'type': 'reset'})
        client_socket.sendall(reset_message)
        received = self._recv_obs(client_socket)
        obs = received['obs']
        client_socket.close()
        return obs, {}
