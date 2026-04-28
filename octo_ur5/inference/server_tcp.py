"""TCP inference server for Octo on UR5 — runs on the GPU machine.

Receives observations from the robot client via TCP, runs model inference,
sends actions back. The robot side uses client_env.py.

Usage:
    python -m octo_ur5.inference.server_tcp \
        --checkpoint_path=./checkpoints/finetuned \
        --checkpoint_step=400000 \
        --port=1242
"""

from datetime import datetime
from functools import partial
import os
import time

from absl import app, flags, logging
import click
import cv2
import imageio
import io
import jax
import jax.numpy as jnp
import numpy as np
import pickle
import socket

from octo.model.octo_model import OctoModel
from octo.utils.train_callbacks import supply_rng

np.set_printoptions(suppress=True)
logging.set_verbosity(logging.WARNING)

FLAGS = flags.FLAGS
flags.DEFINE_string("checkpoint_path", os.environ.get("OCTO_CHECKPOINT_PATH", "./checkpoints/finetuned"),
                     "Path to finetuned checkpoint")
flags.DEFINE_integer("checkpoint_step", int(os.environ.get("OCTO_CHECKPOINT_STEP", "400000")),
                     "Checkpoint step")
flags.DEFINE_integer("port", 1242, "TCP listen port")
flags.DEFINE_string("video_save_path", os.environ.get("OCTO_VIDEO_SAVE_PATH", "./videos"), "Video save directory")
flags.DEFINE_integer("num_steps", 200, "Max rollout steps")
flags.DEFINE_string("language_instruction", "Pick up the cup and the mug, and then put them down",
                     "Default language instruction")


def _recv_all(client_socket):
    data_length = pickle.loads(client_socket.recv(1024))
    received_data = b""
    while len(received_data) < data_length:
        packet = client_socket.recv(1024 * 33)
        if packet == b"EndTransmission":
            break
        received_data += packet
    return pickle.loads(received_data)


def main(_):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(("", FLAGS.port))
    server_socket.listen(5)
    print(f"TCP server listening on port {FLAGS.port}...")

    model = OctoModel.load_pretrained(FLAGS.checkpoint_path, FLAGS.checkpoint_step)

    stats_key = list(model.dataset_statistics.keys())[0]
    logging.info("Using dataset stats key: %s", stats_key)

    def sample_actions(pretrained_model, observations, tasks, rng):
        observations = jax.tree_map(lambda x: x[None], observations)
        actions = pretrained_model.sample_actions(
            observations, tasks, rng=rng,
            unnormalization_statistics=pretrained_model.dataset_statistics[stats_key]["action"],
        )
        return actions[0]

    policy_fn = supply_rng(partial(sample_actions, model))

    im_size = 256
    goal_image = jnp.zeros((im_size, im_size, 3), dtype=np.uint8)
    goal_instruction = FLAGS.language_instruction

    while True:
        text = goal_instruction
        if click.confirm("Current instruction: '{}'. Change?".format(text), default=True):
            text = input("Instruction: ")
        task = model.create_tasks(texts=[text])
        goal_instruction = text
        goal_image = jnp.zeros_like(goal_image)

        input("Press [Enter] to start (waiting for robot client).")

        t1 = time.time()
        client_socket, addr = server_socket.accept()
        print("Robot connected:", addr)
        reset_message = pickle.dumps({'type': 'reset'})
        client_socket.sendall(reset_message)
        received = _recv_all(client_socket)
        obs = received['obs']
        client_socket.close()
        print("Reset time:", time.time() - t1)
        time.sleep(2.0)

        images = []
        goals = []
        t = 0
        while t < FLAGS.num_steps:
            images.append(obs["image_primary"])
            goals.append(goal_image)

            bgr_img = cv2.cvtColor(obs["image_primary"], cv2.COLOR_RGB2BGR)
            cv2.imshow("img_view", bgr_img)
            cv2.waitKey(20)

            action = np.array(policy_fn(obs, task), dtype=np.float32)

            client_socket, addr = server_socket.accept()
            step_message = pickle.dumps({'type': 'step', 'action': action})
            client_socket.sendall(step_message)
            received = _recv_all(client_socket)
            obs = received['obs']
            truncated = received['truncated']
            client_socket.close()
            t += 1

            if truncated:
                break

        if FLAGS.video_save_path:
            os.makedirs(FLAGS.video_save_path, exist_ok=True)
            save_path = os.path.join(FLAGS.video_save_path, f"{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.mp4")
            video = np.concatenate([np.stack(goals), np.stack(images)], axis=1)
            imageio.mimsave(save_path, video, fps=15)


if __name__ == "__main__":
    app.run(main)
