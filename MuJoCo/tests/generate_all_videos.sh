#!/bin/bash
source .venv_wsl/bin/activate

echo "Scanning for trained models without videos..."

find policies/Baseline_Active_Policy -name "*.pkl" | while read pkl; do
  dir=$(dirname "$pkl")
  echo "=================================================="
  echo "Generating video for: $dir"
  echo "=================================================="
  python tests/save_video_kbot.py --path "$pkl"
done

echo "All missing videos generated!"
