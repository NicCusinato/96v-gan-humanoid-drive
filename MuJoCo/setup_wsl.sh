#!/bin/bash

echo "=========================================="
echo " Setting up WSL2 Python Environment for KBot"
echo "=========================================="

# 1. Ensure system dependencies are up to date
echo "Installing system dependencies (requires sudo password)..."
sudo apt-get update
sudo apt-get install -y python3-venv python3-pip python3-dev build-essential

# 2. Create the WSL virtual environment
echo "Creating .venv_wsl..."
python3 -m venv .venv_wsl

# 3. Activate the environment
source .venv_wsl/bin/activate

# 4. Upgrade pip
echo "Upgrading pip..."
pip install --upgrade pip

# 5. Install JAX with CUDA 12 support (CRITICAL for MJX speed)
echo "Installing JAX (with CUDA 12)..."
pip install -U "jax[cuda12]"

# 6. Install LocoMuJoCo dependencies
echo "Installing RL dependencies..."
pip install mujoco-mjx mushroom-rl wandb

# 7. Install LocoMuJoCo in editable mode
echo "Installing LocoMuJoCo..."
cd loco-mujoco
pip install -e .[all]
cd ..

echo "=========================================="
echo " Setup Complete!"
echo " To start training, run the following:"
echo " source .venv_wsl/bin/activate"
echo " python3 tests/train_kbot_rl.py"
echo "=========================================="
