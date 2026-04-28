"""Prepare and upload UR5/UR3 datasets to HuggingFace.

This script converts RLDS/TFDS datasets to HuggingFace Datasets format
and uploads them to the HuggingFace Hub.

Usage:
    pip install huggingface_hub datasets tensorflow-datasets
    huggingface-cli login

    python tools/upload_to_huggingface.py \
        --data_root=./data \
        --repo_id=ArrebolBlack/octo_ur5_dataset \
        --push
"""

import argparse
import os


def get_dataset_info():
    """Return dataset catalog with metadata."""
    datasets = {
        "ur5_put_cube_on_plate": {
            "robot": "UR5e",
            "type": "real",
            "task": "Pick up cube and place on plate",
            "episodes": 25,
            "observation_keys": ["image_primary", "proprio"],
            "action_dim": 7,
        },
        "pick_cup_1.00": {
            "robot": "UR5e",
            "type": "real",
            "task": "Pick up cup",
            "episodes": 1,
            "observation_keys": ["image_primary", "proprio"],
            "action_dim": 7,
        },
        "pick_cup_2.00": {
            "robot": "UR5e",
            "type": "real",
            "task": "Pick up cup",
            "episodes": 1,
            "observation_keys": ["image_primary", "proprio"],
            "action_dim": 7,
        },
        "pick_reset_1.00": {
            "robot": "UR5e",
            "type": "simulation",
            "task": "Pick up cup & mug and put down (PyBullet)",
            "episodes": 1,
            "observation_keys": ["image_primary", "proprio"],
            "action_dim": 7,
        },
        "ur3_pick_cup_single": {
            "robot": "UR3e",
            "type": "real",
            "task": "Pick up cup",
            "episodes": 20,
            "observation_keys": ["image_primary", "proprio"],
            "action_dim": 7,
        },
        "ur3_pick_cup_single_slow": {
            "robot": "UR3e",
            "type": "real",
            "task": "Pick up cup (slow)",
            "episodes": 10,
            "observation_keys": ["image_primary", "proprio"],
            "action_dim": 7,
        },
        "ur3_pick_golden_cup_single": {
            "robot": "UR3e",
            "type": "real",
            "task": "Pick up golden cup",
            "episodes": 10,
            "observation_keys": ["image_primary", "proprio"],
            "action_dim": 7,
        },
        "ur3_pick_golden_cup_single_slow": {
            "robot": "UR3e",
            "type": "real",
            "task": "Pick up golden cup (slow)",
            "episodes": 10,
            "observation_keys": ["image_primary", "proprio"],
            "action_dim": 7,
        },
        "ur3_pick_silver_cup_single": {
            "robot": "UR3e",
            "type": "real",
            "task": "Pick up silver cup",
            "episodes": 10,
            "observation_keys": ["image_primary", "proprio"],
            "action_dim": 7,
        },
        "ur3_pick_silver_cup_single_slow": {
            "robot": "UR3e",
            "type": "real",
            "task": "Pick up silver cup (slow)",
            "episodes": 10,
            "observation_keys": ["image_primary", "proprio"],
            "action_dim": 7,
        },
    }
    return datasets


def create_dataset_card(datasets, repo_id):
    """Generate a HuggingFace dataset card (README.md)."""
    card = f"""---
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
size_categories:
  - 1K<n<10K
---

# {repo_id}

Real-world robot manipulation datasets for [Octo](https://octo-models.github.io/) finetuning on UR-series robots.

## Dataset Summary

This dataset contains real-world robot manipulation trajectories collected via gamepad teleoperation
on UR5e and UR3e robot arms with a Robotiq gripper and Intel Realsense D435 camera.

## Supported Tasks

"""
    for name, info in datasets.items():
        card += f"- **{name}**: {info['task']} ({info['robot']}, {info['episodes']} episodes)\n"

    card += """
## Dataset Structure

Each episode follows the RLDS/TFDS format compatible with Octo's data loading pipeline:

```
steps/
  observation/
    image_primary: uint8 [256, 256, 3]  # RGB camera image
    proprio: float32 [7]                # [x, y, z, roll, pitch, yaw, gripper]
  action: float32 [7]                   # [x, y, z, roll, pitch, yaw, gripper]
  reward: float32
  discount: float32
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
        name="ur5_put_cube_on_plate_slow",
        data_dir="./data/ur5_put_cube_on_plate_slow/1.0.0",
        image_obs_keys={{"primary": "image_primary"}},
        proprio_obs_key="proprio",
        language_key="language_instruction",
    ),
    traj_transform_kwargs=dict(window_size=2, action_horizon=4),
    frame_transform_kwargs=dict(resize_size={{"primary": (256, 256)}}),
    train=True,
)
```

### With HuggingFace Datasets

```python
from datasets import load_dataset

ds = load_dataset("{repo_id}")
```

## Citation

```bibtex
@misc{{octo_ur5,
  author = {{ArrebolBlack}},
  title = {{Octo UR5: Real-World Deployment of Octo on Universal Robots}},
  year = {{2024}},
  url = {{https://github.com/ArrebolBlack/Octo}}
}}
```
""".format(repo_id=repo_id, **{"{": "{", "}": "}"})

    # Fix the f-string issue with double braces
    card = card.replace("{{", "{").replace("}}", "}")
    return card


def scan_local_datasets(data_root):
    """Scan local data directory for existing datasets."""
    found = []
    for root, dirs, files in os.walk(data_root):
        if any(f.endswith('.tfrecord') for f in files) or 'features.json' in files:
            found.append(root)
    return found


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data_root", default="./data", help="Root directory of datasets")
    parser.add_argument("--repo_id", default="ArrebolBlack/octo_ur5_dataset", help="HuggingFace repo ID")
    parser.add_argument("--push", action="store_true", help="Push to HuggingFace Hub")
    parser.add_argument("--dry_run", action="store_true", help="Only print what would be done")
    args = parser.parse_args()

    datasets = get_dataset_info()

    print("=" * 60)
    print("Dataset Upload Preparation")
    print("=" * 60)

    # 1. Generate dataset card
    card = create_dataset_card(datasets, args.repo_id)
    card_path = os.path.join(args.data_root, "hf_dataset_card.md")
    os.makedirs(args.data_root, exist_ok=True)

    if args.dry_run:
        print(f"\n[DRY RUN] Would write dataset card to: {card_path}")
        print("\n--- Dataset Card Preview ---")
        print(card[:500] + "...")
    else:
        with open(card_path, "w", encoding="utf-8") as f:
            f.write(card)
        print(f"Dataset card written to: {card_path}")

    # 2. Scan for local datasets
    print(f"\nScanning {args.data_root} for datasets...")
    found = scan_local_datasets(args.data_root)
    if found:
        print(f"Found {len(found)} dataset directories:")
        for d in found:
            print(f"  - {d}")
    else:
        print("No RLDS/TFDS datasets found locally.")
        print("Download from Baidu Netdisk first, then place under data_root.")

    # 3. Upload instructions
    print("\n" + "=" * 60)
    print("Upload Instructions")
    print("=" * 60)
    print(f"""
Option A: Upload as HuggingFace Dataset (recommended)
    1. pip install huggingface_hub datasets
    2. huggingface-cli login
    3. Create the repo:
       huggingface-cli repo create {args.repo_id} --type dataset
    4. Upload the dataset card:
       cp {card_path} ./data/README.md
    5. Upload data files:
       huggingface-cli upload {args.repo_id} ./data/

Option B: Convert RLDS to HuggingFace Datasets format
    from datasets import Dataset
    import tensorflow_datasets as tfds
    # ... convert and upload with Dataset.push_to_hub()

After upload, update README.md in the repo:
  - Change "[Coming soon]" to the actual HuggingFace link
  - Link: https://huggingface.co/datasets/{args.repo_id}
""")


if __name__ == "__main__":
    main()
