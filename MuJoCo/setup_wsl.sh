#!/bin/bash

echo "=========================================="
echo " Setting up WSL2 Python Environment for KBot"
echo "=========================================="

# 1. Ensure system dependencies are up to date and install Python 3.10
echo "Installing system dependencies and Python 3.10 (requires sudo password)..."
sudo apt-get update
sudo apt-get install -y software-properties-common
sudo add-apt-repository -y ppa:deadsnakes/ppa
sudo apt-get update
sudo apt-get install -y python3.10 python3.10-venv python3.10-dev python3-pip build-essential

# 2. Create the WSL virtual environment inside the project folder
# The --copies flag is CRITICAL here: it forces Python to copy files instead of creating Linux symlinks, 
# which prevents the NTFS "Operation not permitted" error on the Windows drive!
VENV_DIR=".venv_wsl"
echo "Creating Python 3.10 virtual environment at $VENV_DIR..."
python3.10 -m venv --copies $VENV_DIR

if [ ! -f "$VENV_DIR/bin/activate" ]; then
    echo "ERROR: Failed to create the virtual environment!"
    exit 1
fi

# 3. Activate the environment
source $VENV_DIR/bin/activate

# 4. Upgrade pip and core build tools (Pin setuptools to fix Python 3.12+ ImpImporter error)
echo "Upgrading pip, setuptools, and wheel..."
pip install --upgrade pip wheel
pip install "setuptools<70.0.0" numpy

# 5. Install JAX with CUDA 12 support (CRITICAL for MJX speed)
echo "Installing JAX (with CUDA 12)..."
pip install -U "jax[cuda12]"

# 6. Install LocoMuJoCo dependencies
echo "Installing RL dependencies..."
pip install mujoco-mjx mushroom-rl wandb

# 7. Install LocoMuJoCo
echo "Installing LocoMuJoCo (copying to native linux filesystem first to avoid NTFS errors)..."
# Copy to home directory to completely bypass Windows NTFS permission/symlink errors
cp -r loco-mujoco ~/.loco-mujoco-build
cd ~/.loco-mujoco-build
pip install .[all]
cd /mnt/c/96v_gan_humanoid_drive/MuJoCo
rm -rf ~/.loco-mujoco-build

echo "=========================================="
echo " Setup Complete!"
echo " To start training, run the following:"
echo " source .venv_wsl/bin/activate"
echo " python tests/train_kbot_rl.py"
echo "=========================================="
