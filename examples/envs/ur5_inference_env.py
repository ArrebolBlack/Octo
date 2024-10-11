import time

import gym
import numpy as np

import socket
import pickle

import io
from PIL import Image

def decompress_image(compressed_data):
    """
    解压缩图像字节数据

    :param compressed_data: 压缩后的字节数据
    :return: 解压缩后的图像对象
    """
    buffer = io.BytesIO(compressed_data)
    image = Image.open(buffer)
    return np.array(image)


class UR5Gym(gym.Env):

    def __init__(self, port):

        self.im_size = 256

        self.observation_space = gym.spaces.Dict(
            {
                "image_primary": gym.spaces.Box(
                    low=np.zeros((self.im_size, self.im_size, 3)),
                    high=255 * np.ones((self.im_size, self.im_size, 3)),
                    dtype=np.uint8,
                ),
                "proprio": gym.spaces.Box(
                    low=np.ones((7,)) * -3.14, high=np.ones((7,)) * 3.14, dtype=np.float32
                ),
            }
        )
        self.action_space = gym.spaces.Box(
            low=np.zeros((7,)) * -3.14, high=np.ones((7,)) * 3.14, dtype=np.float32
        )

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        self.host = ""
        self.port = port

        self.server_socket.bind((self.host, self.port))

        self.server_socket.listen(5)

        print("等待客户端连接...")


    def step(self, action):
        # 发送step指令和动作给client，并接收返回的数据
        client_socket, addr = self.server_socket.accept()
        print("连接地址：", addr)

        

        step_message = pickle.dumps({'type': 'step', 'action': action})
        client_socket.sendall(step_message)
        data_length = pickle.loads(client_socket.recv(1024))
        received_data = b""
        while len(received_data) < data_length:
        # while True:
            packet = client_socket.recv(1024 * 33)
            if packet == b"EndTransmission":
                print(packet)
                break
            # if packet == b"":
            #     break
            received_data += packet

        print("start loading")
        received_data = pickle.loads(received_data)
        print("收到数据")

        
        obs = received_data['obs']
        # obs["image_primary"] = decompress_image(obs["image_primary"])
        truncated = received_data['truncated']
        client_socket.close()
        return obs, 0, False, truncated, {}

    def reset(self):

        client_socket, addr = self.server_socket.accept()
        print('连接地址：', addr)
        
        # 发送reset指令给client，启动reset，并接收返回的数据
        reset_message = pickle.dumps({'type': 'reset'})
        client_socket.sendall(reset_message)

        data_length = pickle.loads(client_socket.recv(1024))
        received_data = b""
        while len(received_data) < data_length:
        # while True:
            packet = client_socket.recv(1024 * 33)
            
            if packet == b"EndTransmission":
                print(packet)
                break
            # if packet == b"":
            #     break
            received_data += packet
            

        print("start loading")
        received_data = pickle.loads(received_data)
        print("收到数据")

        obs = received_data['obs']
        # obs["image_primary"] = decompress_image(obs["image_primary"])
        # print("obs_image_primary_shape:",obs["image_primary"].shape)
        # print("obs_proprio_shape:",obs["proprio"].shape)

        client_socket.close()

        return obs, {}