
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

FLAGS = flags.FLAGS

flags.DEFINE_string(
    "finetuned_path", "/home/xiaosa/Newpython/Octo/save_big_finetune_result/octo/Trial for finetune_20240813_034547", "Path to finetuned Octo checkpoint directory."
)
# "/home/xiaosa/Newpython/Octo/octo/octo-base-1.5"
# "/home/xiaosa/Newpython/Octo/save_finetuning_checkpoints"
# "/home/xiaosa/Newpython/Octo/save_big_finetune_result/octo/Trial for finetune_20240813_034547"
def main(_):
    # setup wandb for logging
    wandb.init(name="eval_pybullet_big_checkpoint_400timesteps", project="octo")

    # load finetuned model
    logging.info("Loading finetuned model...")
    model = OctoModel.load_pretrained(FLAGS.finetuned_path)

    # make gym environment
    ##################################################################################################################
    # environment needs to implement standard gym interface + return observations of the following form:
    #   obs = {
    #     "image_primary": ...
    #   }
    # it should also implement an env.get_task() function that returns a task dict with goal and/or language instruct.
    #   task = {
    #     "language_instruction": "some string"
    #     "goal": {
    #       "image_primary": ...
    #     }
    #   }
    ##################################################################################################################
    # env = gym.make("aloha-sim-cube-v0")
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


    # wrap env to normalize proprio
    env = NormalizeProprio(env, model.dataset_statistics)

    # add wrappers for history and "receding horizon control", i.e. action chunking
    env = HistoryWrapper(env, horizon=1)
    # env = RHCWrapper(env, exec_horizon=50) # yuanben shi 50
    env = RHCWrapper(env, exec_horizon=4)

    # the supply_rng wrapper supplies a new random key to sample_actions every time it's called
    policy_fn = supply_rng(
        partial(
            model.sample_actions,
            unnormalization_statistics=model.dataset_statistics["action"],
        ),
    )

    # running rollouts
    for _ in range(3):
        obs, info = env.reset()

        # create task specification --> use model utility to create task dict with correct entries
        language_instruction = env.get_task()["language_instruction"]
        task = model.create_tasks(texts=language_instruction)

        # run rollout for 400 steps
        images = [obs["image_primary"][0]]
        episode_return = 0.0
        while len(images) < 400:
            # model returns actions of shape [batch, pred_horizon, action_dim] -- remove batch
            actions = policy_fn(jax.tree_map(lambda x: x[None], obs), task)
            actions = actions[0]

            # step env -- info contains full "chunk" of observations for logging
            # obs only contains observation for final step of chunk
            obs, reward, terminated, truncated, info = env.step(actions)
            images.extend([o["image_primary"][0] for o in info["observations"]])
            episode_return += reward
            if terminated or truncated :
                print("finish!!!")
                break
        print(f"Episode return: {episode_return}")

        # log rollout video to wandb -- subsample temporally 2x for faster logging
        wandb.log(
            {"rollout_video": wandb.Video(np.array(images).transpose(0, 3, 1, 2)[::2])}
        )


if __name__ == "__main__":
    app.run(main)
