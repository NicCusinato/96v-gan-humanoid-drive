#!/bin/bash
# =============================================================================
# Stage 1 Passive Policy Training
# Trains walk and run gaits with alpha-gated PD control.
# Output: policies/Passive_Stage1/<name>/
# =============================================================================

source .venv_wsl/bin/activate
export POLICY_TYPE="Passive Stage1"

BASE_OUT="policies/Passive_Stage1"
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
        --config-path passive \
        --config-name "$config" \
        "hydra.run.dir=$out_dir"
}

echo "Stage 1 Passive Policy | 6 trials | Estimated time: 20-24 hours"
run_trial "walk_02_02" "conf_walk_02_02"
run_trial "walk_07_12" "conf_walk_07_12"
run_trial "walk_07_08" "conf_walk_07_08"
run_trial "run_09_04"  "conf_run_09_04"
run_trial "run_38_03"  "conf_run_38_03"
run_trial "run_16_35"  "conf_run_16_35"

echo ""
echo "All Stage 1 Passive Policy trials complete!"
