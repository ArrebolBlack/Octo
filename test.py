import tensorflow as tf
import tensorflow_datasets as tfds
import numpy as np
import jax
from octo.model.octo_model import OctoModel
import cv2
import mediapy
import tqdm

# 初始化模型
model = OctoModel.load_pretrained("/home/xiaosa/Newpython/Octo/octo/octo-base-1.5")

# 加载数据集
builder = tfds.builder_from_directory('gs://gresearch/robotics/bridge/0.1.0/')
ds = builder.as_dataset(split='train[:1]')

# 处理和加载数据
def process_episode(episode):
    steps = list(episode['steps'])
    images = [cv2.resize(np.array(step['observation']['image']), (256, 256)) for step in steps]
    return images, steps

for episode in ds:
    images, steps = process_episode(episode)
    goal_image = images[-1]
    language_instruction = steps[0]['observation']['natural_language_instruction'].numpy().decode()

    # 推理
    WINDOW_SIZE = 2
    task = model.create_tasks(goals={"image_primary": goal_image[None]})
    pred_actions, true_actions = [], []

    for step in tqdm.trange(len(images) - (WINDOW_SIZE - 1)):
        input_images = np.stack(images[step:step+WINDOW_SIZE])[None]
        observation = {
            'image_primary': input_images,
            'timestep_pad_mask': np.full((1, input_images.shape[1]), True, dtype=bool)
        }
        actions = model.sample_actions(
            observation, 
            task, 
            unnormalization_statistics=model.dataset_statistics["bridge_dataset"]["action"], 
            rng=jax.random.PRNGKey(0)
        )
        actions = actions[0]
        pred_actions.append(actions)
        final_window_step = step + WINDOW_SIZE - 1
        true_actions.append(np.concatenate(
            (
                steps[final_window_step]['action']['world_vector'], 
                steps[final_window_step]['action']['rotation_delta'], 
                np.array(steps[final_window_step]['action']['open_gripper']).astype(np.float32)[None]
            ), axis=-1
        ))

    # 可视化
    pred_actions = np.array(pred_actions).squeeze()
    true_actions = np.array(true_actions).squeeze()
    img_strip = np.concatenate(np.array(images[::3]), axis=1)
    fig, axs = plt.subplots(len(ACTION_DIM_LABELS) + 1, 1, figsize=(45, 10))
    
    for action_dim, action_label in enumerate(ACTION_DIM_LABELS):
        axs[action_dim].plot(pred_actions[:, 0, action_dim], label='predicted action')
        axs[action_dim].plot(true_actions[:, action_dim], label='ground truth')
        axs[action_dim].set_title(action_label)
        axs[action_dim].set_xlabel('Time in one episode')
    
    axs[-1].imshow(img_strip)
    axs[-1].set_xlabel('Time in one episode (subsampled)')
    plt.legend()
    plt.show()
