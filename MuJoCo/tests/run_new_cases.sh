#!/bin/bash
source .venv_wsl/bin/activate

echo "Starting Batch Training for New Cases..."

echo "Training Jump 13_11..."
python tests/train_kbot_amass.py --config-name=conf_jump_13_11

echo "Training Jump 75_01..."
python tests/train_kbot_amass.py --config-name=conf_jump_75_01

echo "Training Jump 75_03..."
python tests/train_kbot_amass.py --config-name=conf_jump_75_03

echo "Training Run 16_35..."
python tests/train_kbot_amass.py --config-name=conf_run_16_35

echo "All new trainings completed!"
