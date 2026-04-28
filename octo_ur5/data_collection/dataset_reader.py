"""Read and inspect recorded RLDS/TFDS trajectory datasets.

Usage:
    python -m octo_ur5.data_collection.dataset_reader --data_dir=./data/my_dataset/1.0.0
"""

import os
import tensorflow as tf
import tensorflow_datasets as tfds

data_dir = os.environ.get("OCTO_UR5_DATA_DIR", "./data/pybullet_ur5_pick_reset_cup_mug/1.0.0")

builder = tfds.builder_from_directory(data_dir)
print(builder.info)
builder.download_and_prepare()

ds = builder.as_dataset(split='train')

for example in ds.take(1):
    for step_data in example['steps']:
        print('Action:', step_data['action'].numpy())
        print('Reward:', step_data['reward'].numpy())
        print('Discount:', step_data['discount'].numpy())
        print('Is first:', step_data['is_first'].numpy())
        print('Is last:', step_data['is_last'].numpy())
        print("language_instruction:", step_data['language_instruction'])
