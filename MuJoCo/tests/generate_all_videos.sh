#!/bin/bash
source .venv_wsl/bin/activate

echo "Scanning for trained models without videos..."

find outputs -name "*.pkl" | while read pkl; do
  dir=$(dirname "$pkl")
  if [ ! -f "$dir/kbot_walk.mp4" ]; then
    echo "=================================================="
    echo "Generating video for: $dir"
    echo "=================================================="
    python tests/save_video_kbot.py --path "$pkl"
  else
    echo "Skipping $dir (video already exists)"
  fi
done

echo "All missing videos generated!"
