from datetime import datetime
from functools import partial
import os
import time

from absl import app, logging
import click
import cv2

import imageio
import jax
import jax.numpy as jnp
import numpy as np

from octo.model.octo_model import OctoModel
from octo.utils.train_callbacks import supply_rng

import socket
import pickle

from PIL import Image
import io

np.set_printoptions(suppress=True)

logging.set_verbosity(logging.WARNING)


##############################################################################
checkpoint_weights_path = "/home/xiaosa/Newpython/Octo/save_finetuning_chechpoints_real_1"
checkpoint_step = 4999
im_size = 256
video_save_path = "/home/xiaosa/Newpython/Octo/video_save_eval"
num_steps = 200

STEP_DURATION = 0.2

# window_size = 2
# action_horizon = 4
show_image = True

##############################################################################

server_ip = "10.8.14.160"
server_port = 8000

# 创建UDP Socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# 绑定IP地址和端口
server_socket.bind((server_ip, server_port))
# 记录客户端地址
client_address = None

def receive_data():
    global client_address
    while True:
        data, client_address = server_socket.recvfrom(65507)
        if data:
            return pickle.loads(data)

def send_data(data):
    if client_address:
        server_socket.sendto(data, client_address)

def decompress_image(compressed_data):
    """
    解压缩图像字节数据

    :param compressed_data: 压缩后的字节数据
    :return: 解压缩后的图像对象
    """
    buffer = io.BytesIO(compressed_data)
    image = Image.open(buffer)
    return np.array(image)
def main(_):
    # set up the robot


    # load models
    model = OctoModel.load_pretrained(
        checkpoint_weights_path,
        checkpoint_step,
    )


    # create policy functions
    def sample_actions(
            pretrained_model: OctoModel,
            observations,
            tasks,
            rng,
    ):
        # add batch dim to observations
        observations = jax.tree_map(lambda x: x[None], observations)
        actions = pretrained_model.sample_actions(
            observations,
            tasks,
            rng=rng,
            unnormalization_statistics=pretrained_model.dataset_statistics[
                "bridge_dataset"
            ]["action"],
        )
        # remove batch dim
        return actions[0]

    policy_fn = supply_rng(
        partial(
            sample_actions,
            model,
            # argmax=FLAGS.deterministic,
            # temperature=FLAGS.temperature,
        )
    )
    # 老版的在仿真环境上的是这样：
    # # the supply_rng wrapper supplies a new random key to sample_actions every time it's called
    # policy_fn = supply_rng(
    #     partial(
    #         model.sample_actions,
    #         unnormalization_statistics=model.dataset_statistics['austin_buds_dataset_converted_externally_to_rlds']["action"],
    #     ),
    # )


    goal_image = jnp.zeros((im_size, im_size, 3), dtype=np.uint8)
    goal_instruction = "Pick up the cup and the mug, and then put them down"


    # goal sampling loop
    while True:


        print("Current instruction: ", goal_instruction)
        text = goal_instruction
        if click.confirm("Take a new instruction?", default=True):
            text = input("Instruction?")
        # Format task for the model
        task = model.create_tasks(texts=[text])
        # For logging purposes
        goal_instruction = text
        goal_image = jnp.zeros_like(goal_image)


        input("Press [Enter] to start.")

        # reset env
        # 发送reset指令给client，启动reset，并接收返回的数据
        reset_message = pickle.dumps({'type': 'reset'})
        send_data(reset_message)
        response = receive_data()
        obs = response['obs']
        obs["image_primary"] = decompress_image(obs["image_primary"])


        time.sleep(2.0)

        # do rollout
        last_tstep = time.time()
        images = []
        goals = []
        t = 0
        
        while t < num_steps:
            if time.time() > last_tstep + STEP_DURATION:
                last_tstep = time.time()

                # save images
                images.append(obs["image_primary"])
                goals.append(goal_image)

                if show_image:
                    bgr_img = cv2.cvtColor(obs["image_primary"], cv2.COLOR_RGB2BGR)
                    cv2.imshow("img_view", bgr_img)
                    cv2.waitKey(20)

                # get action
                forward_pass_time = time.time()
                action = np.array(policy_fn(obs, task), dtype=np.float64)
                print("forward pass time: ", time.time() - forward_pass_time)

                # perform environment step
                start_time = time.time()

                # 发送step指令和动作给client，并接收返回的数据
                step_message = pickle.dumps({'type': 'step', 'action': action})
                send_data(step_message)
                response = receive_data()
                obs = response['obs']
                obs["image_primary"] = decompress_image(obs["image_primary"])
                truncated = response['truncated']

                print("step time: ", time.time() - start_time)

                t += 1

                if truncated:
                    break

        # save video
        if video_save_path is not None:
            os.makedirs(video_save_path, exist_ok=True)
            curr_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            save_path = os.path.join(
                video_save_path,
                f"{curr_time}.mp4",
            )
            video = np.concatenate([np.stack(goals), np.stack(images)], axis=1)
            imageio.mimsave(save_path, video, fps=1.0 / STEP_DURATION * 3)




if __name__ == "__main__":

    app.run(main)