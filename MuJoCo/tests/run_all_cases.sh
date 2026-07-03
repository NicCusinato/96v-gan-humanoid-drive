#!/bin/bash
source .venv_wsl/bin/activate

echo "Starting Batch Training..."

echo "Training Walk 02_02..."
python tests/train_kbot_amass.py --config-name=conf_walk_02_02

echo "Training Walk 07_12..."
python tests/train_kbot_amass.py --config-name=conf_walk_07_12

echo "Training Walk 07_08..."
python tests/train_kbot_amass.py --config-name=conf_walk_07_08

echo "Training Run 09_04..."
python tests/train_kbot_amass.py --config-name=conf_run_09_04

echo "Training Run 16_57..."
python tests/train_kbot_amass.py --config-name=conf_run_16_57

echo "Training Run 38_03..."
python tests/train_kbot_amass.py --config-name=conf_run_38_03

echo "Training Jump 13_13..."
python tests/train_kbot_amass.py --config-name=conf_jump_13_13

echo "Training Jump 16_34..."
python tests/train_kbot_amass.py --config-name=conf_jump_16_34

echo "Training Jump 16_35..."
python tests/train_kbot_amass.py --config-name=conf_jump_16_35

echo "Training Backflip 87_01..."
python tests/train_kbot_amass.py --config-name=conf_backflip_87_01

echo "All trainings completed!"
