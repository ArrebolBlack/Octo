from ml_collections import ConfigDict
from ml_collections.config_dict import FieldReference, placeholder

from octo.utils.spec import ModuleSpec


def get_config(config_string="full,language_conditioned"):
    mode, task = config_string.split(",")
    assert task in ["image_conditioned", "language_conditioned", "multimodal"]
    assert mode in ["full", "head_only", "head_mlp_only"]

    # Fill this in for your own dataset!

    # There should be two image keys
    # first image key should be the third-person view (None if not used)
    # and second image key should be the wrist view (None if not used)


    def create_finetuning_kwargs(name: str, data_dir: str) -> dict:
        return {
            "name": name,
            "data_dir": data_dir,
            "image_obs_keys": {"primary": "image_primary"},
            "proprio_obs_key": "proprio",
            "language_key": "language_instruction",
            "action_proprio_normalization_type": "normal",
            # We want to avoid normalizing the gripper
            "action_normalization_mask": [True, True, True, True, True, True, False],
            "standardize_fn": None,
        }

    data_dirs = (
      # [ f"/home/ziwu/Newpython/Octo_8.27/ur3_datasets/ur3_pick_cup_single_slow/ur3_pick_cup_single_slow_{i}" for i in range(1, 21)]
    # + [f"/home/ziwu/Newpython/Octo_8.27/ur3_datasets/ur3_pick_golden_cup_single_slow/ur3_pick_golden_cup_single_slow_{i}" for i in range(1, 11)]
    # + [f"/home/ziwu/Newpython/Octo_8.27/ur3_datasets/ur3_pick_silver_cup_single_slow/ur3_pick_silver_cup_single_slow_{i}" for i in range(1, 11)]
        [f"/home/ziwu/Newpython/Octo_8.27/ur5_datasets/ur5_put_cube_on_plate_slow/ur5_put_cube_on_plate_slow_{i}" for i in range(1, 11)]
    )
    names = (
      # [f"ur3_pick_cup_single_slow_{i}" for i in range(1, 21)]
    # + [f"ur3_pick_golden_cup_single_slow_{i}" for i in range(1, 11)]
    # + [f"ur3_pick_silver_cup_single_slow_{i}" for i in range(1, 11)]
        [f"ur5_put_cube_on_plate_slow_{i}" for i in range(1, 11)]
    )
    #
    # data_dirs = (
    #         [f"/home/ziwu/Newpython/Octo_8.27/ur3_datasets/ur3_pick_cup_single/ur3_pick_cup_single_{i}" for i in range(1, 21)]
    #         + [f"/home/ziwu/Newpython/Octo_8.27/ur3_datasets/ur3_pick_golden_cup_single/ur3_pick_golden_cup_single_{i}" for i in range(1, 11)]
    #         + [f"/home/ziwu/Newpython/Octo_8.27/ur3_datasets/ur3_pick_silver_cup_single/ur3_pick_silver_cup_single_{i}" for i in range(1, 11)]
    # )
    # names = (
    #         [f"ur3_pick_cup_single_{i}" for i in range(1, 21)]
    #         + [f"ur3_pick_golden_cup_single_{i}" for i in range(1, 11)]
    #         + [f"ur3_pick_silver_cup_single_{i}" for i in range(1, 11)]
    # )
    # 使用生成的字典列表
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
    else:
        raise ValueError("Invalid mode")

    max_steps = FieldReference(1000001)
    window_size = FieldReference(default=2)

    config = dict(
        pretrained_path="/home/ziwu/Newpython/Octo_8.27/octo/octo-base-1.5/",
        pretrained_step=300000,
        # pretrained_path="/home/ziwu/Newpython/Octo_8.27/save_big_finetune_result/octo/finetune on ur5_put_cube_on_plate_20240906_073719",
        # pretrained_path="/data/ziwu/octo/finetune on ur5_put_cube_on_plate_20240903_225003",
        # pretrained_step=500000,
        batch_size=2,
        shuffle_buffer_size=10000,
        num_steps=max_steps,
        log_interval=100,
        # eval_interval=5000,
        eval_interval=1000001,
        save_interval=50000,
        save_dir="/data/ziwu/octo",
        seed=42,
        wandb=dict(
            project="octo", 
            # group=placeholder(str), 
            # entity="yinjiaqi"
        ),
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


                # name="cosine",
                # init_value=3e-4,
                # peak_value=3e-4,
                # warmup_steps=100,
                # decay_steps=max_steps,
                # end_value=3e-4,

                # name="linear",     #ValueError: Unsupported lr schedule: linear
                # init_value=0.0,   # 初始学习率设为0或很小的值
                # peak_value=3e-4,  # 达到的峰值
                # warmup_steps=2000, # 学习率线性增加的步数
                # decay_steps=0,    # 如果不希望衰减，设置为0
                # end_value=3e-4,   # 保持在峰值学习率
            ),
            weight_decay=0.01,
            clip_gradient=1.0,
            frozen_keys=frozen_keys,
            grad_accumulation_steps=None,  # if you are using grad accumulation, you need to adjust max_steps accordingly
        ),
        val_kwargs=dict(
            val_shuffle_buffer_size=1000,
            num_val_batches=2,
        ),
        viz_kwargs=dict(
            eval_batch_size=2,
            trajs_for_metrics=100,
            trajs_for_viz=8,
            samples_per_state=8,
        ),
    )

    if task == "image_conditioned":
        goal_relabeling_strategy = "uniform"
        keep_image_prob = 1.0
    elif task == "language_conditioned":
        goal_relabeling_strategy = None
        keep_image_prob = 0.0
    elif task == "multimodal":
        goal_relabeling_strategy = "uniform"
        keep_image_prob = 0.5
    else:
        raise ValueError("Invalid modality")

    traj_transform_kwargs = dict(
        window_size=window_size,
        action_horizon=4,   # used to be 4, or 50 to see what happen
        goal_relabeling_strategy=goal_relabeling_strategy,
        task_augment_strategy="delete_task_conditioning",
        task_augment_kwargs=dict(
            keep_image_prob=keep_image_prob,
        ),
        # If the default data loading speed is too slow, try these:
        # num_parallel_calls=16,  # for less CPU-intensive ops
    )
    workspace_augment_kwargs = dict(
        random_resized_crop=dict(scale=[0.8, 1.0], ratio=[0.9, 1.1]),
        random_brightness=[0.1],
        random_contrast=[0.9, 1.1],
        random_saturation=[0.9, 1.1],
        random_hue=[0.05],
        augment_order=[
            "random_resized_crop",
            "random_brightness",
            "random_contrast",
            "random_saturation",
            "random_hue",
        ],
    )
    # wrist_augment_kwargs = dict(
    #     random_brightness=[0.1],
    #     random_contrast=[0.9, 1.1],
    #     random_saturation=[0.9, 1.1],
    #     random_hue=[0.05],
    #     augment_order=[
    #         "random_brightness",
    #         "random_contrast",
    #         "random_saturation",
    #         "random_hue",
    #     ],
    # )
    frame_transform_kwargs = dict(
        resize_size={
            "primary": (256, 256),  # workspace (3rd person) camera is at 256x256
            "wrist": (128, 128),  # wrist camera is at 128x128
        },
        image_augment_kwargs=dict(
            primary=workspace_augment_kwargs,
            # wrist=wrist_augment_kwargs,
        ),
    )
    # If the default data loading speed is too slow, try these:
    config[
        "frame_transform_threads"
    ] = 16  # for the most CPU-intensive ops (decoding, resizing, augmenting)

    config["traj_transform_kwargs"] = traj_transform_kwargs
    config["frame_transform_kwargs"] = frame_transform_kwargs
    return ConfigDict(config)
