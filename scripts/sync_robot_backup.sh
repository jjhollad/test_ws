#!/usr/bin/env bash
# Refresh workspace and map copies from $HOME into this repo.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HOME_DIR="${HOME:?}"

RSYNC_EXCLUDES=(
  --exclude 'build/'
  --exclude 'install/'
  --exclude 'log/'
  --exclude '.git/'
)

for ws in create_ws create3_examples_ws sweepbot_ws launch_ws jackal_navigation; do
  if [[ -d "$HOME_DIR/$ws" ]]; then
    echo "Syncing $ws ..."
    rsync -a --delete "${RSYNC_EXCLUDES[@]}" \
      "$HOME_DIR/$ws/" "$REPO_ROOT/workspaces/$ws/"
  else
    echo "Skipping $ws (not found)"
  fi
done

if [[ -d "$HOME_DIR/Arduino" ]]; then
  echo "Syncing Arduino ..."
  rsync -a --delete "$HOME_DIR/Arduino/" "$REPO_ROOT/arduino/"
fi

if [[ -d "$HOME_DIR/maps" ]]; then
  echo "Syncing maps ..."
  rsync -a --delete "$HOME_DIR/maps/" "$REPO_ROOT/backup_maps/EERCsB/"
fi

shopt -s nullglob
map_files=("$HOME_DIR"/EERC_SB*.pgm "$HOME_DIR"/EERC_SB*.yaml)
if ((${#map_files[@]})); then
  rsync -a "${map_files[@]}" "$REPO_ROOT/backup_maps/"
fi

echo "Done. Review with: git status"
