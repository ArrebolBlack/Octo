---
license: mit
task_categories:
  - robotics
tags:
  - robotics
  - ur5
  - ur3
  - octo
  - imitation-learning
  - embodied-ai
  - rlds
size_categories:
  - 1K<n<10K
---

# Octo UR5/UR3 Dataset

Real-world and simulation robot manipulation datasets for [Octo](https://octo-models.github.io/) finetuning on UR-series robots.
Collected via gamepad teleoperation on **UR5e** and **UR3e** with Robotiq gripper and Intel Realsense D435 camera.

## Dataset Summary

| Stat | Value |
|------|-------|
| Total episodes | 98 |
| Total size | ~3.1 GB |
| Format | RLDS / TFDS (TensorFlow Datasets) |
| Observation | RGB image (256×256) + proprio (7-DoF) |
| Action | 7-DoF (x, y, z, roll, pitch, yaw, gripper) |

## UR5 Datasets

| Name | Type | Task | Episodes |
|------|------|------|----------|
| ur5_put_cube_on_plate (1–25) | Real | Pick up cube and place on plate | 25 |
| pick_cup_1.00 | Real | Pick up cup | 1 |
| pick_cup_2.00 | Real | Pick up cup | 1 |
| pick_reset_1.00 | Simulation (PyBullet) | Pick up cup & mug, put down | 1 |

## UR3 Datasets

| Name | Task | Episodes |
|------|------|----------|
| ur3_pick_cup_single (1–20) | Pick up cup | 20 |
| ur3_pick_cup_single_slow (1–10) | Pick up cup (slow) | 10 |
| ur3_pick_golden_cup_single (1–10) | Pick up golden cup | 10 |
| ur3_pick_golden_cup_single_slow (1–10) | Pick up golden cup (slow) | 10 |
| ur3_pick_silver_cup_single (1–10) | Pick up silver cup | 10 |
| ur3_pick_silver_cup_single_slow (1–10) | Pick up silver cup (slow) | 10 |

## Dataset Structure

Each dataset follows the RLDS/TFDS format, compatible with [Octo](https://github.com/rail-berkeley/octo)'s data loading pipeline:

```
episode/
  steps/
    observation/
      image_primary: uint8 [256, 256, 3]   # RGB camera image (JPEG encoded)
      proprio: float32 [7]                 # [x, y, z, roll, pitch, yaw, gripper]
    action: float32 [7]                    # [x, y, z, roll, pitch, yaw, gripper]
    reward: float32
    discount: float64
    language_instruction: string
    is_first: bool
    is_last: bool
```

## Usage

### With Octo Training Pipeline

```python
from octo.data.dataset import make_single_dataset

dataset = make_single_dataset(
    dataset_kwargs=dict(
        name="ur5_put_cube_on_plate_1",
        data_dir="./ur5_put_on/ur5_put_cube_on_plate/ur5_put_cube_on_plate_1",
        image_obs_keys={"primary": "image_primary"},
        proprio_obs_key="proprio",
        language_key="language_instruction",
    ),
    traj_transform_kwargs=dict(window_size=2, action_horizon=4),
    frame_transform_kwargs=dict(resize_size={"primary": (256, 256)}),
    train=True,
)
```

### Multi-dataset Finetuning

```python
from octo.data.dataset import make_interleaved_dataset

# Point data_dir to each sub-dataset
dataset_kwargs = [
    {"name": f"ur5_put_cube_on_plate_{i}",
     "data_dir": f"./ur5_put_on/ur5_put_cube_on_plate/ur5_put_cube_on_plate_{i}",
     "image_obs_keys": {"primary": "image_primary"},
     "proprio_obs_key": "proprio",
     "language_key": "language_instruction"}
    for i in range(1, 26)
]

dataset = make_interleaved_dataset(
    dataset_kwargs,
    traj_transform_kwargs=dict(window_size=2, action_horizon=4),
    frame_transform_kwargs=dict(resize_size={"primary": (256, 256)}),
    train=True,
)
```

## Citation

```bibtex
@misc{octo_ur5,
  author = {ArrebolBlack},
  title = {Octo UR5: Real-World Deployment of Octo on Universal Robots},
  year = {2024},
  url = {https://github.com/ArrebolBlack/Octo}
}

@inproceedings{octo_2023,
    title={Octo: An Open-Source Generalist Robot Policy},
    author = {{Octo Model Team} and Dibya Ghosh and Homer Walke and Karl Pertsch and Kevin Black and Oier Mees and Sudeep Dasari and Joey Hejna and Charles Xu and Jianlan Luo and Tobias Kreiman and {You Liang} Tan and Pannag Sanketi and Quan Vuong and Ted Xiao and Dorsa Sadigh and Chelsea Finn and Sergey Levine},
    booktitle = {Proceedings of Robotics: Science and Systems},
    year = {2024},
}
```
