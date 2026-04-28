"""Upload dataset card (README.md) to HuggingFace repo root.

Usage:
    python tools/upload_dataset_card.py --repo_id=JiaqiYin/octo_ur5_dataset
"""

import argparse
from huggingface_hub import HfApi


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo_id", default="JiaqiYin/octo_ur5_dataset")
    args = parser.parse_args()

    api = HfApi()
    api.upload_file(
        path_or_fileobj="tools/hf_dataset_card.md",
        path_in_repo="README.md",
        repo_id=args.repo_id,
        repo_type="dataset",
    )
    print(f"Dataset card uploaded to https://huggingface.co/datasets/{args.repo_id}")


if __name__ == "__main__":
    main()
