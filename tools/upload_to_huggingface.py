"""Upload UR5/UR3 datasets to HuggingFace Hub.

Scans the downloaded data directory, generates a dataset card,
and uploads all TFRecord datasets to HuggingFace.

Usage:
    pip install huggingface_hub
    huggingface-cli login

    # Dry run first (scans + generates card, no upload)
    python tools/upload_to_huggingface.py \
        --data_root="D:/Octo数据集和数采代码(UR5)" --dry_run

    # Upload
    python tools/upload_to_huggingface.py \
        --data_root="D:/Octo数据集和数采代码(UR5)" \
        --repo_id=ArrebolBlack/octo_ur5_dataset --push
"""

import argparse
import json
import os
from pathlib import Path

DATASET_CATALOG = [
    # UR5
    {"name": "ur5_put_cube_on_plate", "path": "ur5_put_on/ur5_put_cube_on_plate",
     "robot": "UR5e", "type": "real", "task": "Pick up cube and place on plate", "episodes": 25},
    {"name": "pick_cup_1.00", "path": "pick_cup_1.00",
     "robot": "UR5e", "type": "real", "task": "Pick up cup", "episodes": 1},
    {"name": "pick_cup_2.00", "path": "pick_cup_2.00",
     "robot": "UR5e", "type": "real", "task": "Pick up cup", "episodes": 1},
    {"name": "pick_reset_1.00", "path": "pick_reset_1.00",
     "robot": "UR5e", "type": "simulation", "task": "Pick up cup & mug, put down (PyBullet)", "episodes": 1},
    # UR3
    {"name": "ur3_pick_cup_single", "path": "ur3_pick_cup_single",
     "robot": "UR3e", "type": "real", "task": "Pick up cup", "episodes": 20},
    {"name": "ur3_pick_cup_single_slow", "path": "ur3_pick_cup_single_slow",
     "robot": "UR3e", "type": "real", "task": "Pick up cup (slow)", "episodes": 10},
    {"name": "ur3_pick_golden_cup_single", "path": "ur3_pick_golden_cup_single",
     "robot": "UR3e", "type": "real", "task": "Pick up golden cup", "episodes": 10},
    {"name": "ur3_pick_golden_cup_single_slow", "path": "ur3_pick_golden_cup_single_slow",
     "robot": "UR3e", "type": "real", "task": "Pick up golden cup (slow)", "episodes": 10},
    {"name": "ur3_pick_silver_cup_single", "path": "ur3_pick_silver_cup_single",
     "robot": "UR3e", "type": "real", "task": "Pick up silver cup", "episodes": 10},
    {"name": "ur3_pick_silver_cup_single_slow", "path": "ur3_pick_silver_cup_single_slow",
     "robot": "UR3e", "type": "real", "task": "Pick up silver cup (slow)", "episodes": 10},
]


def scan_datasets(data_root):
    """Scan data_root for datasets matching the catalog."""
    found = []
    for entry in DATASET_CATALOG:
        full_path = os.path.join(data_root, entry["path"])
        if os.path.isdir(full_path):
            # Verify it has TFRecord or sub-datasets
            has_data = False
            for root, dirs, files in os.walk(full_path):
                if any(f.endswith('.tfrecord') for f in files):
                    has_data = True
                    break
                if 'features.json' in files:
                    has_data = True
                    break
            if has_data:
                found.append({**entry, "full_path": full_path})
    return found


def get_dataset_size(path):
    """Get total size of dataset directory."""
    total = 0
    for root, dirs, files in os.walk(path):
        for f in files:
            total += os.path.getsize(os.path.join(root, f))
    return total


def create_dataset_card(datasets, repo_id):
    """Generate HuggingFace dataset card."""
    lines = []
    lines.append("---")
    lines.append("license: mit")
    lines.append("task_categories:")
    lines.append("  - robotics")
    lines.append("tags:")
    lines.append("  - robotics")
    lines.append("  - ur5")
    lines.append("  - ur3")
    lines.append("  - octo")
    lines.append("  - imitation-learning")
    lines.append("  - embodied-ai")
    lines.append("  - rlds")
    lines.append("---")
    lines.append("")
    lines.append(f"# {repo_id.split('/')[-1]}")
    lines.append("")
    lines.append("Real-world and simulation robot manipulation datasets for "
                 "[Octo](https://octo-models.github.io/) finetuning on UR-series robots.")
    lines.append("Collected via gamepad teleoperation on UR5e/UR3e with Robotiq gripper and Intel Realsense D435.")
    lines.append("")
    lines.append("## Datasets")
    lines.append("")

    total_episodes = 0
    total_size = 0

    lines.append("| Name | Robot | Type | Task | Episodes |")
    lines.append("|------|-------|------|------|----------|")
    for ds in datasets:
        size = get_dataset_size(ds["full_path"])
        total_size += size
        total_episodes += ds["episodes"]
        lines.append(f"| {ds['name']} | {ds['robot']} | {ds['type']} | {ds['task']} | {ds['episodes']} |")

    lines.append("")
    lines.append(f"**Total: {total_episodes} episodes, ~{total_size / (1024**3):.1f}GB**")
    lines.append("")
    lines.append("## Dataset Structure (RLDS/TFDS format)")
    lines.append("")
    lines.append("```")
    lines.append("steps/")
    lines.append("  observation/")
    lines.append("    image_primary: uint8 [256, 256, 3]  # RGB camera image")
    lines.append("    proprio: float32 [7]                # [x, y, z, roll, pitch, yaw, gripper]")
    lines.append("  action: float32 [7]                   # [x, y, z, roll, pitch, yaw, gripper]")
    lines.append("  reward: float32")
    lines.append("  discount: float64")
    lines.append("  language_instruction: string")
    lines.append("  is_first: bool")
    lines.append("  is_last: bool")
    lines.append("```")
    lines.append("")
    lines.append("## Usage with Octo")
    lines.append("")
    lines.append("```python")
    lines.append("from octo.data.dataset import make_single_dataset")
    lines.append("")
    lines.append("dataset = make_single_dataset(")
    lines.append("    dataset_kwargs=dict(")
    lines.append("        name='ur5_put_cube_on_plate_1',")
    lines.append("        data_dir='./data/ur5_put_on/ur5_put_cube_on_plate/ur5_put_cube_on_plate_1',")
    lines.append("        image_obs_keys={'primary': 'image_primary'},")
    lines.append("        proprio_obs_key='proprio',")
    lines.append("        language_key='language_instruction',")
    lines.append("    ),")
    lines.append("    traj_transform_kwargs=dict(window_size=2, action_horizon=4),")
    lines.append("    frame_transform_kwargs=dict(resize_size={'primary': (256, 256)}),")
    lines.append("    train=True,")
    lines.append(")")
    lines.append("```")
    lines.append("")
    lines.append("## Citation")
    lines.append("")
    lines.append("```bibtex")
    lines.append("@misc{octo_ur5,")
    lines.append("  author = {ArrebolBlack},")
    lines.append("  title = {Octo UR5: Real-World Deployment of Octo on Universal Robots},")
    lines.append("  year = {2024},")
    lines.append("  url = {https://github.com/ArrebolBlack/Octo}")
    lines.append("}")
    lines.append("```")

    return "\n".join(lines)


def do_upload(datasets, data_root, repo_id):
    """Upload datasets to HuggingFace Hub."""
    from huggingface_hub import HfApi

    api = HfApi()

    # Create repo if not exists
    try:
        api.create_repo(repo_id=repo_id, repo_type="dataset", exist_ok=True)
        print(f"Repo ensured: {repo_id}")
    except Exception as e:
        print(f"Error creating repo: {e}")
        return

    # Upload dataset card
    card = create_dataset_card(datasets, repo_id)
    card_path = os.path.join(data_root, "README.md")
    with open(card_path, "w", encoding="utf-8") as f:
        f.write(card)
    print(f"Dataset card written to: {card_path}")

    # Upload data directories
    for ds in datasets:
        print(f"\nUploading: {ds['name']} from {ds['path']}")
        api.upload_large_folder(
            folder_path=ds["full_path"],
            repo_id=repo_id,
            repo_type="dataset",
            path_in_repo=ds["path"],
        )
        print(f"  Done: {ds['name']}")

    print(f"\nAll uploads complete!")
    print(f"View at: https://huggingface.co/datasets/{repo_id}")


def main():
    parser = argparse.ArgumentParser(description="Upload UR5/UR3 datasets to HuggingFace")
    parser.add_argument("--data_root", required=True, help="Root directory of downloaded datasets")
    parser.add_argument("--repo_id", default="ArrebolBlack/octo_ur5_dataset", help="HuggingFace repo ID")
    parser.add_argument("--push", action="store_true", help="Actually upload to HuggingFace Hub")
    parser.add_argument("--dry_run", action="store_true", help="Scan and show summary without uploading")
    args = parser.parse_args()

    print("=" * 60)
    print("HuggingFace Dataset Upload Tool")
    print("=" * 60)

    # Scan
    datasets = scan_datasets(args.data_root)
    if not datasets:
        print(f"\nNo datasets found in: {args.data_root}")
        print("Expected directory structure:")
        print("  data_root/")
        print("    ur5_put_on/ur5_put_cube_on_plate/...")
        print("    ur3_pick_cup_single/...")
        print("    pick_reset_1.00/...")
        return

    print(f"\nFound {len(datasets)} datasets:")
    total_size = 0
    total_episodes = 0
    for ds in datasets:
        size = get_dataset_size(ds["full_path"])
        total_size += size
        total_episodes += ds["episodes"]
        print(f"  {ds['name']:40s}  {ds['episodes']:3d} ep  {size/(1024**2):8.1f} MB  [{ds['type']}]")
    print(f"\n  Total: {total_episodes} episodes, {total_size/(1024**3):.2f} GB")

    if args.dry_run:
        # Generate and show dataset card
        card = create_dataset_card(datasets, args.repo_id)
        print("\n" + "-" * 60)
        print("Dataset Card Preview (first 1500 chars):")
        print("-" * 60)
        print(card[:1500])
        if len(card) > 1500:
            print(f"... ({len(card)} chars total)")
        print("\n[DRY RUN] No files uploaded. Run with --push to upload.")
        return

    if not args.push:
        print("\nRun with --push to actually upload, or --dry_run to preview.")
        return

    do_upload(datasets, args.data_root, args.repo_id)


if __name__ == "__main__":
    main()
