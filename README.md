# Octo UR5: Real-World Deployment of Octo on Universal Robots

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![HuggingFace Dataset](https://img.shields.io/badge/Dataset-HuggingFace-blue)](https://huggingface.co/datasets/JiaqiYin/octo_ur5_dataset)

A complete open-source pipeline for deploying the [Octo](https://octo-models.github.io/) embodied intelligence model on **UR-series robots** (UR5/UR3). Includes simulation environments, real-robot teleoperation, data collection, finetuning, and distributed inference — all in pure Python.

[中文文档](README_zh.md) | [Original Octo README](#octo-upstream)

> **Based on**: [rail-berkeley/octo](https://github.com/rail-berkeley/octo)
> **Tested on**: UR5e + Robotiq gripper + Intel Realsense D435, UR3e
>
> **Datasets**: 10 datasets, 98 episodes (~3.3GB) — [HuggingFace](https://huggingface.co/datasets/JiaqiYin/octo_ur5_dataset) | [Baidu Netdisk](https://pan.baidu.com/s/5aeV1f-_CITH-ZKrlKf5MqQ)

---

## What's New (vs. Octo Upstream)

| Feature | Octo Upstream | This Repo |
|---------|:---:|:---:|
| Real UR5/UR3 robot control (RTDE) | ❌ | ✅ |
| Servo gripper control (Modbus RTU) | ❌ | ✅ |
| Gamepad teleoperation (6-DoF + gripper) | ❌ | ✅ |
| Realsense camera integration | ❌ | ✅ |
| PyBullet UR5 simulation | ❌ | ✅ |
| RLDS/TFDS data recording pipeline | ❌ | ✅ |
| Distributed inference (GPU ↔ Robot via TCP/UDP) | ❌ | ✅ |
| UR-series finetune configs (single + multi-dataset) | ❌ | ✅ |
| Real-world UR5/UR3 datasets | ❌ | ✅ |

---

## Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                      Complete Pipeline                       │
│                                                              │
│  ┌──────────┐    ┌──────────────┐    ┌────────────────────┐  │
│  │ Teleop   │───>│ Record Data  │───>│ Train & Deploy     │  │
│  │ (Gamepad)│    │ (envlogger)  │    │ (Octo finetune)    │  │
│  │          │    │              │    │                    │  │
│  │ UR5/UR3  │    │ RLDS/TFDS    │    │ ┌──────────────┐   │  │
│  │ Realsense│    │ Dataset      │    │ │ Inference    │   │  │
│  │ Gripper  │    │              │    │ │ Server       │   │  │
│  └──────────┘    └──────────────┘    │ │ (GPU)        │   │  │
│                                      │ └──────────────┘   │  │
│                                      │       TCP/UDP │    │  │
│                                      │ ┌───────────┼──┐   │  │
│                                      │ │ Robot Client │   │  │
│                                      │ │ (edge)       │   │  │
│                                      │ └──────────────┘   │  │
│                                      └────────────────────┘  │
└──────────────────────────────────────────────────────────────┘
```

---

## Repository Structure

```
octo/                          # Octo model core (upstream, with dataset config additions)
octo_ur5/                      # ★ All original contributions
├── real/                      # Real robot: UR5 controller, gripper, camera, gamepad
│   ├── robot_controller.py   # RTDE control + servoL + gripper integration
│   ├── gripper_controller.py # Serial Modbus RTU gripper (0–1000 range)
│   ├── utilities.py          # Realsense D435 + Pygame gamepad controller
│   └── real_ur5_env.py       # Gym wrapper for real UR5 + camera + gripper
├── sim/                       # Simulation environments
│   ├── pybullet_ur5_env.py   # PyBullet UR5 Robotiq env (external dep)
│   └── robopal_ur5_env.py    # Robopal UR5e env (experimental)
├── inference/                 # Distributed inference: GPU server ↔ robot client
│   ├── server_tcp.py         # TCP inference server (runs on GPU machine)
│   ├── server_udp.py         # UDP inference server (lower latency)
│   ├── client_env.py         # Socket-based Gym env (GPU side, used by eval scripts)
│   └── client_robot_tcp.py   # Robot-side TCP client (runs on robot machine)
├── data_collection/           # Data collection pipeline
│   ├── teleop_main.py        # Gamepad teleoperation main loop
│   ├── dataset_recorder.py   # Record to RLDS/TFDS via envlogger
│   └── dataset_reader.py     # Inspect stored trajectory datasets
├── configs/                   # Finetune configuration files
│   ├── finetune_config.py    # Single-dataset config (UR5)
│   └── finetune_config_multi_dataset.py  # Multi-dataset (UR5 + UR3)
├── scripts/                   # Training & evaluation entry points
│   ├── finetune_simple.py    # Standalone finetuning script
│   ├── eval_sim.py           # PyBullet simulation evaluation
│   ├── eval_real.py          # Real robot evaluation (direct)
│   └── eval_real_socket.py   # Real robot evaluation (socket)
├── gym_wrappers.py            # HistoryWrapper, RHCWrapper, TemporalEnsembleWrapper
├── gym2dmenv.py               # Gym → dm_env adapter (for envlogger)
└── tests/                     # Hardware connection tests
    ├── test_camera.py
    ├── test_joystick.py
    └── test_ur_rtde.py
examples/                      # Octo upstream examples (preserved as-is)
scripts/                       # Octo upstream training scripts
docs/                          # Documentation and presentations
```

---

## Installation

### 1. Clone & Install Octo

```bash
git clone https://github.com/ArrebolBlack/Octo.git
cd Octo

conda create -n octo python=3.10
conda activate octo
pip install -e .
pip install -r requirements.txt

# GPU (CUDA 11)
pip install --upgrade "jax[cuda11_pip]==0.4.20" -f https://storage.googleapis.com/jax-releases/jax_cuda_releases.html
```

### 2. Install Real Robot Dependencies

```bash
pip install pyrealsense2 pygame pyserial scipy

# RTDE library (Universal Robots real-time communication)
pip install ur-rtde
```

### 3. Install Simulation Dependencies (Optional)

```bash
pip install pybullet envlogger tensorflow-datasets dm-env
# PyBullet UR5 Robotiq env: add pybullet_ur5_robotiq to PYTHONPATH
```

### 4. Configure Environment Variables

```bash
export UR5_ROBOT_IP="192.168.4.100"       # Robot IP
export UR5_GRIPPER_PORT="COM5"            # Gripper serial port
export OCTO_PRETRAINED_PATH="./checkpoints/octo-base-1.5"
export OCTO_UR5_DATA_DIR="./data/my_dataset/1.0.0"
export OCTO_SAVE_DIR="./checkpoints/finetuned"
```

---

## Quick Start: Complete Pipeline

### Step 1 — Test Hardware

```bash
python -m octo_ur5.tests.test_ur_rtde     # Test UR5 RTDE connection
python -m octo_ur5.tests.test_camera      # Test Realsense camera
python -m octo_ur5.tests.test_joystick    # Test gamepad
```

### Step 2 — Teleoperate & Collect Data

**Real robot teleoperation:**
```bash
python -m octo_ur5.data_collection.teleop_main
```

**Record simulation data to RLDS/TFDS format:**
```bash
python -m octo_ur5.data_collection.dataset_recorder \
    --data_dir=./data/output --num_episodes=10
```

**Inspect recorded data:**
```bash
python -m octo_ur5.data_collection.dataset_reader
```

### Step 3 — Finetune Octo

**Simple finetuning:**
```bash
python -m octo_ur5.scripts.finetune_simple \
    --pretrained_path=./checkpoints/octo-base-1.5 \
    --data_dir=./data/my_dataset/1.0.0 \
    --save_dir=./checkpoints/finetuned \
    --batch_size=8 --num_steps=5000
```

**Using Octo's advanced training script:**
```bash
python scripts/finetune.py \
    --config=octo_ur5/configs/finetune_config.py:full,language_conditioned
```

**Multi-dataset (UR5 + UR3):**
```bash
export OCTO_UR5_DATA_ROOT="./data"
python scripts/finetune_multi_dataset.py \
    --config=octo_ur5/configs/finetune_config_multi_dataset.py:full,language_conditioned
```

### Step 4 — Evaluate & Deploy

**Simulation:**
```bash
python -m octo_ur5.scripts.eval_sim \
    --finetuned_path=./checkpoints/finetuned --num_episodes=3
```

**Real robot (direct — single machine):**
```bash
python -m octo_ur5.scripts.eval_real \
    --checkpoint_path=./checkpoints/finetuned \
    --checkpoint_step=400000 --port=1293
```

**Real robot (distributed — GPU server + robot client):**

On GPU server:
```bash
python -m octo_ur5.inference.server_tcp \
    --checkpoint_path=./checkpoints/finetuned \
    --checkpoint_step=400000 --port=1242
```

On robot machine:
```bash
python -m octo_ur5.inference.client_robot_tcp
# Configure: export OCTO_SERVER_IP=10.8.14.160  OCTO_SERVER_PORT=1242
```

---

## Datasets

Real-world and simulation datasets in RLDS/TFDS format (compatible with Octo's data pipeline).
All real-world data collected via gamepad teleoperation.

### UR5 Datasets

| Dataset | Type | Task | Episodes | Size |
|---------|------|------|----------|------|
| ur5_put_cube_on_plate (1–25) | Real | Pick cube, place on plate | 25 | 1.2G |
| pick_cup_1.00 | Real | Pick up cup | 1 | 22M |
| pick_cup_2.00 | Real | Pick up cup | 1 | 28M |
| pick_reset_1.00 | Sim (PyBullet) | Pick up cup & mug, put down | 1 | 65M |

### UR3 Datasets

| Dataset | Task | Episodes | Size |
|---------|------|----------|------|
| ur3_pick_cup_single (1–20) | Pick up cup | 20 | 447M |
| ur3_pick_cup_single_slow (1–10) | Pick up cup (slow) | 10 | 450M |
| ur3_pick_golden_cup_single (1–10) | Pick up golden cup | 10 | 198M |
| ur3_pick_golden_cup_single_slow (1–10) | Pick up golden cup (slow) | 10 | 522M |
| ur3_pick_silver_cup_single (1–10) | Pick up silver cup | 10 | 195M |
| ur3_pick_silver_cup_single_slow (1–10) | Pick up silver cup (slow) | 10 | 107M |

**Total: ~3.3GB, 98 episodes**

### Access

- **HuggingFace**: [https://huggingface.co/datasets/JiaqiYin/octo_ur5_dataset](https://huggingface.co/datasets/JiaqiYin/octo_ur5_dataset)
- **Baidu Netdisk**: [https://pan.baidu.com/s/5aeV1f-_CITH-ZKrlKf5MqQ](https://pan.baidu.com/s/5aeV1f-_CITH-ZKrlKf5MqQ)

---

## External Dependencies

| Library | Purpose | Install |
|---------|---------|---------|
| [pybullet_ur5_robotiq](https://github.com/stepjam/PyBulletrobots) | PyBullet UR5 Robotiq simulation environment | Add to PYTHONPATH |
| [robopal](https://github.com/None-JX/robopal) | UR5e simulation (experimental, optional) | `pip install robopal` |
| [ur-rtde](https://github.com/UniversalRobots/RTDE_Python_Client_Library) | UR RTDE real-time communication | `pip install ur-rtde` |

---

## Documentation

- [**Octo Reproduction Presentation**](docs/Octo复现.pdf) — 11-page technical slides covering model architecture, pipeline design, dataset details, and UR3/UR5 experiment results (in Chinese).

## Octo Upstream

This repository includes the full Octo model codebase from [rail-berkeley/octo](https://github.com/rail-berkeley/octo). For Octo-specific documentation (model architecture, pretraining, original examples), see:

- [Octo Examples](examples/)
- [Octo Training Scripts](scripts/)
- [Octo Model Documentation](https://octo-models.github.io/)

---

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

## Contact

- **WeChat**: wxid_aotp6u5i4n522
- **Email**: yjqhit@gmail.com

## License

MIT License (inherited from [Octo](https://github.com/rail-berkeley/octo)).
