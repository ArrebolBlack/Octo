# Octo UR5：Octo 具身智能模型在优傲机器人上的真机部署

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![HuggingFace Dataset](https://img.shields.io/badge/Dataset-HuggingFace-blue)](https://huggingface.co/datasets/JiaqiYin/octo_ur5_dataset)

基于 [Octo](https://octo-models.github.io/) 具身智能模型，在 **UR 系列机器人**（UR5/UR3）上实现完整部署的开源 Pipeline。包含仿真环境、真机遥操作、数据采集、模型微调和分布式推理——全部纯 Python 实现。

[English](README.md) | [原版 Octo README](#octo-上游项目)

> **基于**：[rail-berkeley/octo](https://github.com/rail-berkeley/octo)
> **已验证平台**：UR5e + Robotiq 夹爪 + Intel Realsense D435，UR3e
>
> **数据集**：10 个数据集，98 个 episode（约 3.3GB）— [HuggingFace](https://huggingface.co/datasets/JiaqiYin/octo_ur5_dataset) | [百度网盘](https://pan.baidu.com/s/5aeV1f-_CITH-ZKrlKf5MqQ)

---

## 相比 Octo 原仓库的新贡献

| 功能 | Octo 原仓库 | 本仓库 |
|------|:---:|:---:|
| UR5/UR3 真机控制（RTDE 协议） | ❌ | ✅ |
| 伺服夹爪控制（Modbus RTU 串口） | ❌ | ✅ |
| 手柄遥操作（6自由度 + 夹爪） | ❌ | ✅ |
| Realsense 相机集成 | ❌ | ✅ |
| PyBullet UR5 仿真环境 | ❌ | ✅ |
| RLDS/TFDS 数据录制 Pipeline | ❌ | ✅ |
| 分布式推理（GPU ↔ 机器人，TCP/UDP） | ❌ | ✅ |
| UR 系列微调配置（单数据集 + 多数据集） | ❌ | ✅ |
| 真机 UR5/UR3 数据集 | ❌ | ✅ |

---

## 系统架构

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

## 安装

### 1. 克隆并安装 Octo

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

### 2. 安装真机依赖

```bash
pip install pyrealsense2 pygame pyserial scipy

# RTDE 库（优傲机器人实时通信）
pip install ur-rtde
# 或者将仓库中自带的 RTDE_Python_Client_Library/ 目录添加到 PYTHONPATH
```

### 3. 安装仿真依赖（可选）

```bash
pip install pybullet envlogger tensorflow-datasets dm-env
```

### 4. 配置环境变量

```bash
export UR5_ROBOT_IP="192.168.4.100"
export UR5_GRIPPER_PORT="COM5"              # Linux: /dev/ttyUSB0
export OCTO_PRETRAINED_PATH="./checkpoints/octo-base-1.5"
export OCTO_UR5_DATA_DIR="./data/my_dataset/1.0.0"
export OCTO_SAVE_DIR="./checkpoints/finetuned"
```

---

## 快速开始：完整流程

### 第 1 步 — 测试硬件连接

```bash
python -m octo_ur5.tests.test_ur_rtde     # 测试 UR5 RTDE 连接
python -m octo_ur5.tests.test_camera      # 测试 Realsense 相机
python -m octo_ur5.tests.test_joystick    # 测试手柄
```

### 第 2 步 — 遥操作与数据采集

**真机遥操作：**
```bash
python -m octo_ur5.data_collection.teleop_main
```

**录制仿真数据（RLDS/TFDS 格式）：**
```bash
python -m octo_ur5.data_collection.dataset_recorder \
    --data_dir=./data/output --num_episodes=10
```

**查看已录制的轨迹：**
```bash
python -m octo_ur5.data_collection.dataset_reader
```

### 第 3 步 — 微调 Octo

**简单微调：**
```bash
python -m octo_ur5.scripts.finetune_simple \
    --pretrained_path=./checkpoints/octo-base-1.5 \
    --data_dir=./data/my_dataset/1.0.0 \
    --save_dir=./checkpoints/finetuned \
    --batch_size=8 --num_steps=5000
```

**使用 Octo 高级训练脚本：**
```bash
python scripts/finetune.py \
    --config=octo_ur5/configs/finetune_config.py:full,language_conditioned
```

### 第 4 步 — 评估与部署

**仿真环境评估：**
```bash
python -m octo_ur5.scripts.eval_sim \
    --finetuned_path=./checkpoints/finetuned --num_episodes=3
```

**真机评估（直连——单机模式）：**
```bash
python -m octo_ur5.scripts.eval_real \
    --checkpoint_path=./checkpoints/finetuned \
    --checkpoint_step=400000 --port=1293
```

**真机评估（分布式——GPU 服务器 + 机器人客户端）：**

GPU 服务器端：
```bash
python -m octo_ur5.inference.server_tcp \
    --checkpoint_path=./checkpoints/finetuned \
    --checkpoint_step=400000 --port=1242
```

机器人端：
```bash
python -m octo_ur5.inference.client_robot_tcp
# 配置: export OCTO_SERVER_IP=10.8.14.160  OCTO_SERVER_PORT=1242
```

---

## 数据集

真机和仿真数据集，RLDS/TFDS 格式（兼容 Octo 数据加载管线）。所有真机数据通过手柄遥操作采集。

### UR5 数据集

| 数据集 | 类型 | 任务 | Episode 数 | 大小 |------|
| ur5_put_cube_on_plate (1–25) | 真机 | 抓取方块放到盘子上 | 25 | 1.2G |
| pick_cup_1.00 | 真机 | 抓取杯子 | 1 | 22M |
| pick_cup_2.00 | 真机 | 抓取杯子 | 1 | 28M |
| pick_reset_1.00 | 仿真 (PyBullet) | 抓取杯子和马克杯并放下 | 1 | 65M |

### UR3 数据集

| 数据集 | 任务 | Episode 数 | 大小 |
|--------|------|-----------|------|
| ur3_pick_cup_single (1–20) | 抓取杯子 | 20 | 447M |
| ur3_pick_cup_single_slow (1–10) | 抓取杯子（慢速） | 10 | 450M |
| ur3_pick_golden_cup_single (1–10) | 抓取金杯子 | 10 | 198M |
| ur3_pick_golden_cup_single_slow (1–10) | 抓取金杯子（慢速） | 10 | 522M |
| ur3_pick_silver_cup_single (1–10) | 抓取银杯子 | 10 | 195M |
| ur3_pick_silver_cup_single_slow (1–10) | 抓取银杯子（慢速） | 10 | 107M |

**总计：约 3.3GB，98 个 episode**

### 获取方式

- **HuggingFace**：[https://huggingface.co/datasets/JiaqiYin/octo_ur5_dataset](https://huggingface.co/datasets/JiaqiYin/octo_ur5_dataset)
- **百度网盘**：[https://pan.baidu.com/s/5aeV1f-_CITH-ZKrlKf5MqQ](https://pan.baidu.com/s/5aeV1f-_CITH-ZKrlKf5MqQ)

---

## 外部依赖

| 库 | 用途 |
|----|------|---------|
| pybullet_ur5_robotiq | PyBullet UR5 Robotiq 仿真环境 | 添加到 PYTHONPATH |
| robopal | UR5e 仿真（实验性，可选） | `pip install robopal` |
| ur-rtde | UR 系列机器人 RTDE 实时通信 | `pip install ur-rtde`（或使用自带的 `RTDE_Python_Client_Library/`） |

---

## 文档

- [**Octo 复现技术报告**](docs/Octo复现.pdf) — 11 页技术报告，涵盖模型架构、Pipeline 设计、数据集详情、UR3/UR5 实验结果与分析。

## Octo 上游项目

本仓库包含 [rail-berkeley/octo](https://github.com/rail-berkeley/octo) 的完整模型代码。Octo 相关文档请参阅：

- [Octo 示例](examples/)
- [Octo 训练脚本](scripts/)
- [Octo 官方文档](https://octo-models.github.io/)

---

## 引用

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

## 联系方式

- **微信**：wxid_aotp6u5i4n522
- **邮箱**：yjqhit@gmail.com

## 许可证

MIT License（继承自 [Octo](https://github.com/rail-berkeley/octo)）。
