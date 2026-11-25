#!/usr/bin/env bash
# Jetson Orin Nano 에서 실행: best.pt → TensorRT FP16 엔진
# 사용법: ./export_engine.sh [model_dir]
set -euo pipefail
dir="${1:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../model" && pwd)}"
cd "$dir"
yolo export model=best.pt format=engine half=True imgsz=640 device=0 simplify=True workspace=2
ls -lh best.engine
