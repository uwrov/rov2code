#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   ./run_da3_view.sh /path/to/images   # generate new point cloud, then open
#   ./run_da3_view.sh #open pregenerated scene

# This will only work on my machine; good luck! :)
VENV_DIR="${VENV_DIR:-/home/imants/UWROV/Photogrammetry/.venv-rocm}"
source "$VENV_DIR/bin/activate"

EXPORT_DIR="$PWD/output"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VIEWER_SCRIPT="${VIEWER_SCRIPT:-$SCRIPT_DIR/pointcloud_annotator.py}"

MODEL_DIR="${MODEL_DIR:-depth-anything/DA3NESTED-GIANT-LARGE-1.1}"
PROCESS_RES="${PROCESS_RES:-800}"
NUM_MAX_POINTS="${NUM_MAX_POINTS:-1000000}"
CONF_THRESH_PERCENTILE="${CONF_THRESH_PERCENTILE:-40}"

if [[ "$#" -eq 1 ]]; then
    IMAGE_DIR="$(realpath "$1")"

    echo "Input images: $IMAGE_DIR"
    echo "Output dir:   $EXPORT_DIR"
    echo "Model:        $MODEL_DIR"
    echo "Process res:  $PROCESS_RES"
    echo

    da3 images "$IMAGE_DIR" \
      --model-dir "$MODEL_DIR" \
      --export-dir "$EXPORT_DIR" \
      --export-format glb \
      --device cpu \
      --process-res "$PROCESS_RES" \
      --num-max-points "$NUM_MAX_POINTS" \
      --conf-thresh-percentile "$CONF_THRESH_PERCENTILE" \
      --auto-cleanup

GLB_PATH="$(find "$EXPORT_DIR" -maxdepth 1 -type f -name '*.glb' | sort | head -n 1)"

if [[ -z "$GLB_PATH" ]]; then
    echo "No .glb file found in: $EXPORT_DIR"
    echo "Run with an image directory first:"
    echo "  $0 /path/to/images"
    exit 1
fi

echo
echo "Opening point cloud:"
echo "$GLB_PATH"
echo

python "$VIEWER_SCRIPT" "$GLB_PATH"