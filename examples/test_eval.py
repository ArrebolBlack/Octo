
from functools import partial
import sys

import os 
from absl import app, flags, logging
import jax
import numpy as np
import wandb

# sys.path.append("/home/xiaosa/Newpython/Octo/act")
sys.path.append("/home/xiaosa/Newpython/Octo/pybullet_ur5_robotiq")

from env import ClutteredTouchPick
from robot import UR5Robotiq85
from utilities import YCBModels, Camera

from octo.model.octo_model import OctoModel
from octo.utils.gym_wrappers import HistoryWrapper, NormalizeProprio, RHCWrapper
from octo.utils.train_callbacks import supply_rng


def main(_):

    ycb_models = YCBModels(
        os.path.join('./data/ycb', '**', 'textured-decmp.obj'),
    )
    camera = Camera((1, 1, 1),
                    (0, 0, 0),
                    (0, 0, 1),
                    0.1, 5, (320, 320), 40)
    # camera = None
    # robot = Panda((0, 0.5, 0), (0, 0, math.pi))
    robot = UR5Robotiq85((0, 0.5, 0), (0, 0, 0))
    env = ClutteredTouchPick(robot, ycb_models, camera, vis=True)

    env.reset()

    fake_dict = {"action": {"mask": [True, True, True, True, True, True, False], "max": [0.20985263586044312, 0.0, 0.5052631497383118, 1.0576841831207275, 1.570796251296997, 1.5707963705062866, 0.08500000089406967], "mean": [0.07639770209789276, 0.0, 0.3351764976978302, 0.9656370878219604, 1.5641708374023438, 1.5641708374023438, 0.05087554082274437], "min": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "p01": [0.0, 0.0, 0.18421052396297455, 0.0, 1.570796251296997, 1.5707963705062866, 0.0], "p99": [0.20985263586044312, 0.0, 0.5052631497383118, 1.0576841831207275, 1.570796251296997, 1.5707963705062866, 0.08500000089406967], "std": [0.09545142948627472, 0.0, 0.12297630310058594, 0.29170742630958557, 0.10181869566440582, 0.10181869566440582, 0.03779086843132973]}, "num_trajectories": 1, "num_transitions": 237, "proprio": {"max": [0.2108050286769867, 0.009423800744116306, 0.5044047832489014, 1.5986989736557007, 1.5707963705062866, 2.1117024421691895, 0.08500000089406967], "mean": [0.07666341215372086, 0.0003233261813875288, 0.3369702696800232, 0.6848666667938232, 1.5655367374420166, 1.2541215419769287, 0.05123418569564819], "min": [-0.04996852949261665, -0.0038023940287530422, 0.1834133118391037, -1.5041940212249756, 1.5572313070297241, -1.5704619884490967, 0.0], "p01": [-0.0005875853239558637, -0.0025430654641240835, 0.1834133118391037, 0.0, 1.5577753782272339, 0.07718642801046371, 0.0], "p99": [0.21070796251296997, 0.004326520953327417, 0.5027512311935425, 1.5805643796920776, 1.5707963705062866, 2.0935657024383545, 0.08500000089406967], "std": [0.09544650465250015, 0.0012956846039742231, 0.12159231305122375, 0.7560833096504211, 0.005691567435860634, 0.7480436563491821, 0.0377095527946949]}}
    # wrap env to normalize proprio
    env = NormalizeProprio(env, fake_dict)
    obs, info = env.reset()
    print(obs, info)

    # add wrappers for history and "receding horizon control", i.e. action chunking
    env = HistoryWrapper(env, horizon=1)
    env = RHCWrapper(env, exec_horizon=50)

    # the supply_rng wrapper supplies a new random key to sample_actions every time it's called
    # policy_fn = supply_rng(
    #     partial(
    #         model.sample_actions,
    #         unnormalization_statistics=model.dataset_statistics["action"],
    #     ),
    # )
    obs, info = env.reset()
    print(obs, info)

# ########
#     # running rollouts
#     for _ in range(3):
#         obs, info = env.reset()

#         # create task specification --> use model utility to create task dict with correct entries
#         language_instruction = env.get_task()["language_instruction"]
#         # task = model.create_tasks(texts=language_instruction)

#         # run rollout for 400 steps
#         images = [obs["image_primary"][0]]
#         episode_return = 0.0
#         while len(images) < 400:
#             # model returns actions of shape [batch, pred_horizon, action_dim] -- remove batch
#             # actions = policy_fn(jax.tree_map(lambda x: x[None], obs), task)
#             actions = actions[0]

#             # step env -- info contains full "chunk" of observations for logging
#             # obs only contains observation for final step of chunk
#             obs, reward, done, info = env.step(actions)
#             images.extend([o["image_primary"][0] for o in info["observations"]])
#             episode_return += reward
#             if done :
#                 break
#         print(f"Episode return: {episode_return}")

#         # log rollout video to wandb -- subsample temporally 2x for faster logging
#         wandb.log(
#             {"rollout_video": wandb.Video(np.array(images).transpose(0, 3, 1, 2)[::2])}
#         )


if __name__ == "__main__":
    app.run(main)
