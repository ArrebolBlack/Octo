"""Multi-dataset finetune config for Octo on UR-series robots.

Supports UR5 and UR3 datasets. Set OCTO_UR5_DATA_ROOT to point to your
dataset root directory containing per-task subdirectories.

Environment variables:
    OCTO_PRETRAINED_PATH  - path to pretrained Octo checkpoint
    OCTO_UR5_DATA_ROOT    - root directory containing dataset subdirectories
    OCTO_SAVE_DIR         - path to save finetuning checkpoints
"""

import os
from ml_collections import ConfigDict
from ml_collections.config_dict import FieldReference

from octo.utils.spec import ModuleSpec


def get_config(config_string="full,language_conditioned"):
    mode, task = config_string.split(",")
    assert task in ["image_conditioned", "language_conditioned", "multimodal"]
    assert mode in ["full", "head_only", "head_mlp_only"]

    data_root = os.environ.get("OCTO_UR5_DATA_ROOT", "./data")

    def create_finetuning_kwargs(name, data_dir):
        return {
            "name": name,
            "data_dir": data_dir,
            "image_obs_keys": {"primary": "image_primary"},
            "proprio_obs_key": "proprio",
            "language_key": "language_instruction",
            "action_proprio_normalization_type": "normal",
            "action_normalization_mask": [True, True, True, True, True, True, False],
            "standardize_fn": None,
        }

    # UR5 datasets
    ur5_task = "ur5_put_cube_on_plate_slow"
    data_dirs = [os.path.join(data_root, "ur5_datasets", f"{ur5_task}/{ur5_task}_{i}") for i in range(1, 11)]
    names = [f"{ur5_task}_{i}" for i in range(1, 11)]

    # Uncomment to include UR3 datasets:
    # ur3_tasks = [
    #     ("ur3_pick_cup_single_slow", 20),
    #     ("ur3_pick_golden_cup_single_slow", 10),
    #     ("ur3_pick_silver_cup_single_slow", 10),
    # ]
    # for task_name, count in ur3_tasks:
    #     data_dirs += [os.path.join(data_root, "ur3_datasets", f"{task_name}/{task_name}_{i}") for i in range(1, count + 1)]
    #     names += [f"{task_name}_{i}" for i in range(1, count + 1)]

    FINETUNING_KWARGS = [
        create_finetuning_kwargs(names[i], data_dir)
        for i, data_dir in enumerate(data_dirs)
    ]

    if mode == "full":
        frozen_keys = None
    elif mode == "head_only":
        frozen_keys = ("octo_transformer.*",)
    elif mode == "head_mlp_only":
        frozen_keys = (
            "octo_transformer.*",
            "heads_*.map_head.probe",
            "heads_*.map_head.MultiHeadDotProductAttention_0.*",
        )

    max_steps = FieldReference(1000001)
    window_size = FieldReference(default=2)

    config = dict(
        pretrained_path=os.environ.get("OCTO_PRETRAINED_PATH", "./checkpoints/octo-base-1.5"),
        pretrained_step=300000,
        batch_size=2,
        shuffle_buffer_size=10000,
        num_steps=max_steps,
        log_interval=100,
        eval_interval=1000001,
        save_interval=50000,
        save_dir=os.environ.get("OCTO_SAVE_DIR", "./checkpoints/finetuned"),
        seed=42,
        wandb=dict(project="octo"),
        dataset_kwargs=FINETUNING_KWARGS,
        modality=task,
        finetuning_mode=mode,
        window_size=window_size,
        optimizer=dict(
            learning_rate=dict(
                name="cosine",
                init_value=0.0,
                peak_value=3e-4,
                warmup_steps=2000,
                decay_steps=max_steps,
                end_value=0.0,
            ),
            weight_decay=0.01,
            clip_gradient=1.0,
            frozen_keys=frozen_keys,
            grad_accumulation_steps=None,
        ),
        val_kwargs=dict(val_shuffle_buffer_size=1000, num_val_batches=2),
        viz_kwargs=dict(eval_batch_size=2, trajs_for_metrics=100, trajs_for_viz=8, samples_per_state=8),
    )

    if task == "image_conditioned":
        goal_relabeling_strategy, keep_image_prob = "uniform", 1.0
    elif task == "language_conditioned":
        goal_relabeling_strategy, keep_image_prob = None, 0.0
    elif task == "multimodal":
        goal_relabeling_strategy, keep_image_prob = "uniform", 0.5

    traj_transform_kwargs = dict(
        window_size=window_size,
        action_horizon=4,
        goal_relabeling_strategy=goal_relabeling_strategy,
        task_augment_strategy="delete_task_conditioning",
        task_augment_kwargs=dict(keep_image_prob=keep_image_prob),
    )

    workspace_augment_kwargs = dict(
        random_resized_crop=dict(scale=[0.8, 1.0], ratio=[0.9, 1.1]),
        random_brightness=[0.1],
        random_contrast=[0.9, 1.1],
        random_saturation=[0.9, 1.1],
        random_hue=[0.05],
        augment_order=["random_resized_crop", "random_brightness", "random_contrast", "random_saturation", "random_hue"],
    )

    frame_transform_kwargs = dict(
        resize_size={"primary": (256, 256), "wrist": (128, 128)},
        image_augment_kwargs=dict(primary=workspace_augment_kwargs),
    )
    config["frame_transform_threads"] = 16
    config["traj_transform_kwargs"] = traj_transform_kwargs
    config["frame_transform_kwargs"] = frame_transform_kwargs
    return ConfigDict(config)
