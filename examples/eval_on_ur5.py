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
from octo.utils.gym_wrappers import HistoryWrapper, TemporalEnsembleWrapper, RHCWrapper

from envs.ur5_inference_env import UR5Gym

np.set_printoptions(suppress=True)

logging.set_verbosity(logging.WARNING)

##############################################################################
# checkpoint_weights_path = "/home/ziwu/Newpython/Octo_8.27/save_big_finetune_result/octo/finetune on ur5_put_cube_on_plate_20240902_184909"
checkpoint_weights_path = "/data/ziwu/octo/octo/finetune on ur5_put_cube_on_plate_20240912_021917"
checkpoint_step = 400000
im_size = 256
video_save_path = "/home/ziwu/Newpython/Octo_8.27/video_save_eval"
num_steps = 200

STEP_DURATION = 0.2

window_size = 2
action_horizon = 4
show_image = True

##############################################################################


def main(_):
    # set up the robot env
    env = UR5Gym(port=1293)

    # wrap the robot environment
    env = HistoryWrapper(env, window_size)
    env = TemporalEnsembleWrapper(env, action_horizon)
    # switch TemporalEnsembleWrapper with RHCWrapper for receding horizon control
    # env = RHCWrapper(env, action_horizon)

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
            unnormalization_statistics=pretrained_model.dataset_statistics["ur5_put_cube_on_plate_slow_1"]["action"],
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
    goal_instruction = "put cube on plate"
        # "Pick up the cup and put it down"

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
        obs, _ = env.reset()
        print("env reset!")
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
                images.append(obs["image_primary"][-1])
                goals.append(goal_image)

                if show_image:
                    bgr_img = cv2.cvtColor(obs["image_primary"][-1], cv2.COLOR_RGB2BGR)
                    cv2.imshow("img_view", bgr_img)
                    cv2.waitKey(20)

                print("start get action")
                # get action
                forward_pass_time = time.time()
                action = np.array(policy_fn(obs, task), dtype=np.float32)
                print("forward pass time: ", time.time() - forward_pass_time)
                # print("action:",action)

                # perform environment step
                start_time = time.time()
                obs, _, _, truncated, _ = env.step(action)
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