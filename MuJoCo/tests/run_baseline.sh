#!/bin/bash
# ============================================================
# Baseline Active Policy Training
# Trains all 12 motion primitives with zero energy penalty.
# Each model saves to: policies/Baseline_Active_Policy/<name>/
# ============================================================

source .venv_wsl/bin/activate
export POLICY_TYPE="Baseline Active"

BASE_OUT="policies/Baseline_Active_Policy"
mkdir -p "$BASE_OUT"

run_trial() {
    local name=$1
    local config=$2
    local out_dir="$BASE_OUT/$name"
    echo ""
    echo "======================================================"
    echo " Training: $name"
    echo " Config:   $config"
    echo " Output:   $out_dir"
    echo "======================================================"
    mkdir -p "$out_dir"
    python tests/train_kbot_amass.py \
        --config-path baseline \
        --config-name "$config" \
        "hydra.run.dir=$out_dir"
}

echo "Starting Baseline Active Policy Training..."
echo "Total: 12 trials | Estimated time: 16-18 hours"
echo ""

run_trial "walk_02_02"     "conf_walk_02_02"
run_trial "walk_07_12"     "conf_walk_07_12"
run_trial "walk_07_08"     "conf_walk_07_08"
run_trial "run_09_04"      "conf_run_09_04"
run_trial "run_16_57"      "conf_run_16_57"
run_trial "run_38_03"      "conf_run_38_03"
run_trial "run_16_35"      "conf_run_16_35"
run_trial "jump_13_13"     "conf_jump_13_13"
run_trial "jump_13_11"     "conf_jump_13_11"
run_trial "jump_75_01"     "conf_jump_75_01"
run_trial "jump_75_03"     "conf_jump_75_03"
run_trial "backflip_87_01" "conf_backflip_87_01"

echo ""
echo "======================================================"
echo " All Baseline Active Policy training completed!"
echo " Find your models in: $BASE_OUT/"
echo "======================================================"
