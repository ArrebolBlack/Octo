"""Evaluate a finetuned Octo model on a real UR5 robot via socket (TCP).

This is the server side — runs on the GPU machine. The robot side uses
octo_ur5.inference.client_env.py to connect.

Adapted from Octo's examples/04_eval_finetuned_on_robot.py for UR5 deployment.

Usage:
    python -m octo_ur5.scripts.eval_real_socket \
        --checkpoint_path=./checkpoints/my_model \
        --checkpoint_step=400000 \
        --port=1293
"""

from datetime import datetime
from functools import partial
import os
import time

from absl import app, flags, logging
import click
import cv2
import imageio
import jax
import jax.numpy as jnp
import numpy as np

from octo.model.octo_model import OctoModel
from octo_ur5.gym_wrappers import HistoryWrapper, TemporalEnsembleWrapper
from octo_ur5.inference.client_env import UR5Gym
from octo.utils.train_callbacks import supply_rng

np.set_printoptions(suppress=True)
logging.set_verbosity(logging.WARNING)

FLAGS = flags.FLAGS
flags.DEFINE_string("checkpoint_path", os.environ.get("OCTO_CHECKPOINT_PATH", "./checkpoints/octo"),
                     "Path to finetuned checkpoint directory")
flags.DEFINE_integer("checkpoint_step", int(os.environ.get("OCTO_CHECKPOINT_STEP", "400000")),
                     "Checkpoint step to load")
flags.DEFINE_integer("im_size", 256, "Image size")
flags.DEFINE_integer("port", 1293, "TCP port for robot communication")
flags.DEFINE_string("video_save_path", os.environ.get("OCTO_VIDEO_SAVE_PATH", "./videos"), "Directory to save videos")
flags.DEFINE_integer("num_timesteps", 120, "Max timesteps per rollout")
flags.DEFINE_integer("window_size", 2, "Observation history length")
flags.DEFINE_integer("action_horizon", 4, "Action chunk length for temporal ensembling")
flags.DEFINE_string("dataset_stats_key", None, "Dataset stats key (auto-detected if None)")


def main(_):
    env = UR5Gym(port=FLAGS.port)
    env = HistoryWrapper(env, FLAGS.window_size)
    env = TemporalEnsembleWrapper(env, FLAGS.action_horizon)

    model = OctoModel.load_pretrained(FLAGS.checkpoint_path, FLAGS.checkpoint_step)

    stats_key = FLAGS.dataset_stats_key
    if stats_key is None:
        stats_key = list(model.dataset_statistics.keys())[0]
        logging.info("Auto-detected dataset stats key: %s", stats_key)

    def sample_actions(pretrained_model, observations, tasks, rng):
        observations = jax.tree_map(lambda x: x[None], observations)
        actions = pretrained_model.sample_actions(
            observations, tasks, rng=rng,
            unnormalization_statistics=pretrained_model.dataset_statistics[stats_key]["action"],
        )
        return actions[0]

    policy_fn = supply_rng(partial(sample_actions, model))

    goal_image = jnp.zeros((FLAGS.im_size, FLAGS.im_size, 3), dtype=np.uint8)
    goal_instruction = ""

    while True:
        modality = click.prompt("Language or goal image?", type=click.Choice(["l", "g"]))

        if modality == "g":
            logging.warning("Goal image mode requires WidowX setup. Using language mode instead.")
            text = input("Instruction: ")
            task = model.create_tasks(texts=[text])
            goal_instruction = text
            goal_image = jnp.zeros_like(goal_image)
        elif modality == "l":
            text = goal_instruction
            if click.confirm("Current instruction: '{}'. Change?".format(text), default=True):
                text = input("Instruction: ")
            task = model.create_tasks(texts=[text])
            goal_instruction = text
            goal_image = jnp.zeros_like(goal_image)

        input("Press [Enter] to start.")
        obs, _ = env.reset()
        time.sleep(2.0)

        images = []
        goals = []
        t = 0
        while t < FLAGS.num_timesteps:
            images.append(obs["image_primary"][-1])
            goals.append(goal_image)

            bgr_img = cv2.cvtColor(obs["image_primary"][-1], cv2.COLOR_RGB2BGR)
            cv2.imshow("img_view", bgr_img)
            cv2.waitKey(20)

            action = np.array(policy_fn(obs, task), dtype=np.float32)
            obs, _, _, truncated, _ = env.step(action)
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
