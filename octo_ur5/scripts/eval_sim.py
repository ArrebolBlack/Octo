"""Evaluate a finetuned Octo model in PyBullet UR5 simulation.

Requires the external `pybullet_ur5_robotiq` library.

Usage:
    python -m octo_ur5.scripts.eval_sim \
        --finetuned_path=./checkpoints/my_model \
        --num_episodes=3
"""

from functools import partial
import os

from absl import app, flags, logging
import jax
import numpy as np
import wandb

# External PyBullet UR5 simulation library
from env import ClutteredTouchPick
from robot import UR5Robotiq85
from utilities import YCBModels, Camera

from octo.model.octo_model import OctoModel
from octo.utils.gym_wrappers import HistoryWrapper, NormalizeProprio, RHCWrapper
from octo.utils.train_callbacks import supply_rng

FLAGS = flags.FLAGS
flags.DEFINE_string("finetuned_path", os.environ.get("OCTO_CHECKPOINT_PATH", "./checkpoints/octo"),
                     "Path to finetuned Octo checkpoint")
flags.DEFINE_integer("num_episodes", 3, "Number of evaluation episodes")
flags.DEFINE_integer("max_steps", 400, "Max steps per episode")


def main(_):
    wandb.init(name="eval_pybullet_ur5", project="octo")
    logging.info("Loading finetuned model...")
    model = OctoModel.load_pretrained(FLAGS.finetuned_path)

    ycb_models = YCBModels(os.path.join('./data/ycb', '**', 'textured-decmp.obj'))
    camera = Camera((1, 1, 1), (0, 0, 0), (0, 0, 1), 0.1, 5, (320, 320), 40)
    robot = UR5Robotiq85((0, 0.5, 0), (0, 0, 0))
    env = ClutteredTouchPick(robot, ycb_models, camera, vis=True)
    env.reset()

    env = NormalizeProprio(env, model.dataset_statistics)
    env = HistoryWrapper(env, horizon=1)
    env = RHCWrapper(env, exec_horizon=4)

    stats_key = list(model.dataset_statistics.keys())[0]
    policy_fn = supply_rng(
        partial(model.sample_actions,
                unnormalization_statistics=model.dataset_statistics[stats_key]["action"]),
    )

    for ep in range(FLAGS.num_episodes):
        obs, info = env.reset()
        language_instruction = env.get_task()["language_instruction"]
        task = model.create_tasks(texts=language_instruction)

        images = [obs["image_primary"][0]]
        episode_return = 0.0
        while len(images) < FLAGS.max_steps:
            actions = policy_fn(jax.tree_map(lambda x: x[None], obs), task)
            actions = actions[0]
            obs, reward, terminated, truncated, info = env.step(actions)
            images.extend([o["image_primary"][0] for o in info["observations"]])
            episode_return += reward
            if terminated or truncated:
                break

        print(f"Episode {ep} return: {episode_return}")
        wandb.log({"rollout_video": wandb.Video(np.array(images).transpose(0, 3, 1, 2)[::2])})


if __name__ == "__main__":
    app.run(main)
