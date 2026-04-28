"""Finetune Octo on a custom UR5 dataset (single camera + proprio, 7-DoF action).

Usage:
    python -m octo_ur5.scripts.finetune_simple \
        --pretrained_path=./checkpoints/octo-base-1.5 \
        --data_dir=./data/pybullet_ur5_pick_reset_cup_mug/1.0.0 \
        --save_dir=./checkpoints/finetuned \
        --batch_size=8
"""

import os
from absl import app, flags, logging
import flax
import jax
import optax
import tensorflow as tf
import tqdm
import wandb

from octo.data.dataset import make_single_dataset
from octo.model.components.action_heads import L1ActionHead
from octo.model.components.tokenizers import LowdimObsTokenizer
from octo.model.octo_model import OctoModel
from octo.utils.jax_utils import initialize_compilation_cache
from octo.utils.spec import ModuleSpec
from octo.utils.train_utils import (
    freeze_weights,
    merge_params,
    process_text,
    TrainState,
)

FLAGS = flags.FLAGS

flags.DEFINE_string("pretrained_path", os.environ.get("OCTO_PRETRAINED_PATH", "./checkpoints/octo-base-1.5"),
                     "Path to pre-trained Octo checkpoint")
flags.DEFINE_string("data_dir", os.environ.get("OCTO_UR5_DATA_DIR", "./data/pybullet_ur5_pick_reset_cup_mug/1.0.0"),
                     "Path to finetuning dataset (RLDS format)")
flags.DEFINE_string("save_dir", os.environ.get("OCTO_SAVE_DIR", "./checkpoints/finetuned"),
                     "Directory for saving finetuning checkpoints")
flags.DEFINE_integer("batch_size", 8, "Batch size")
flags.DEFINE_integer("num_steps", 5000, "Number of training steps")
flags.DEFINE_bool("freeze_transformer", False, "Freeze pre-trained transformer weights")
flags.DEFINE_string("dataset_name", "pybullet_ur5_pick_reset_cup_mug", "Dataset name for RLDS")
flags.DEFINE_string("language_key", "language_instruction", "Language key in dataset")


def main(_):
    assert FLAGS.batch_size % jax.device_count() == 0, "Batch size must be divisible by device count."

    initialize_compilation_cache()
    tf.config.set_visible_devices([], "GPU")

    wandb.init(name="finetune_pybullet_ur5", project="octo")

    logging.info("Loading pre-trained model...")
    pretrained_model = OctoModel.load_pretrained(FLAGS.pretrained_path)

    logging.info("Loading finetuning dataset...")
    dataset = make_single_dataset(
        dataset_kwargs=dict(
            name=FLAGS.dataset_name,
            data_dir=FLAGS.data_dir,
            image_obs_keys={"primary": "image_primary"},
            proprio_obs_key="proprio",
            language_key=FLAGS.language_key,
        ),
        traj_transform_kwargs=dict(window_size=2, action_horizon=4),
        frame_transform_kwargs=dict(resize_size={"primary": (256, 256)}),
        train=True,
    )
    train_data_iter = (
        dataset.repeat().unbatch().shuffle(10000).batch(FLAGS.batch_size).iterator()
    )

    text_processor = pretrained_model.text_processor

    def process_batch(batch):
        batch = process_text(batch, text_processor)
        del batch["dataset_name"]
        return batch

    train_data_iter = map(process_batch, train_data_iter)
    example_batch = next(train_data_iter)

    config = pretrained_model.config
    del config["model"]["observation_tokenizers"]["wrist"]
    config["model"]["observation_tokenizers"]["proprio"] = ModuleSpec.create(
        LowdimObsTokenizer, n_bins=256, bin_type="normal", low=-3.14, high=3.14, obs_keys=["proprio"],
    )
    config["model"]["heads"]["action"] = ModuleSpec.create(
        L1ActionHead, action_horizon=4, action_dim=7, readout_key="readout_action",
    )

    logging.info("Updating model for new observation & action space...")
    model = OctoModel.from_config(
        config, example_batch, text_processor, verbose=True,
        dataset_statistics=dataset.dataset_statistics,
    )
    merged_params = merge_params(model.params, pretrained_model.params)
    model = model.replace(params=merged_params)
    del pretrained_model

    learning_rate = optax.join_schedules(
        [optax.linear_schedule(0, 3e-5, 100), optax.constant_schedule(3e-5)], [100]
    )
    tx = optax.adamw(learning_rate)
    frozen_keys = model.config["optimizer"]["frozen_keys"]
    if FLAGS.freeze_transformer:
        frozen_keys.append("BlockTransformer_0")
    tx = freeze_weights(tx, model.params, frozen_keys)
    train_state = TrainState.create(rng=jax.random.PRNGKey(1234), model=model, tx=tx)

    def loss_fn(params, batch, rng, train=True):
        bound_module = model.module.bind({"params": params}, rngs={"dropout": rng})
        transformer_embeddings = bound_module.octo_transformer(
            batch["observation"], batch["task"],
            batch["observation"]["timestep_pad_mask"], train=train,
        )
        action_loss, action_metrics = bound_module.heads["action"].loss(
            transformer_embeddings, batch["action"],
            batch["observation"]["timestep_pad_mask"], batch["action_pad_mask"], train=train,
        )
        return action_loss, action_metrics

    @jax.jit
    def train_step(state, batch):
        rng, dropout_rng = jax.random.split(state.rng)
        (loss, info), grads = jax.value_and_grad(loss_fn, has_aux=True)(
            state.model.params, batch, dropout_rng, train=True
        )
        return state.apply_gradients(grads=grads, rng=rng), info

    logging.info("Starting finetuning...")
    for i in tqdm.tqdm(range(FLAGS.num_steps), total=FLAGS.num_steps, dynamic_ncols=True):
        batch = next(train_data_iter)
        train_state, update_info = train_step(train_state, batch)
        if (i + 1) % 100 == 0:
            update_info = jax.device_get(update_info)
            wandb.log(flax.traverse_util.flatten_dict({"training": update_info}, sep="/"), step=i)
        if (i + 1) % 1000 == 0:
            train_state.model.save_pretrained(step=i, checkpoint_path=FLAGS.save_dir)


if __name__ == "__main__":
    app.run(main)
