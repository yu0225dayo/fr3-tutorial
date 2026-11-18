#!/bin/bash
# しきい値の倍率を、動かしたまま変える（関節モード・fairino モデル）。デモ用で、実機の設定ではない。
# 使い方: ./set_threshold.sh 倍率    例: ./set_threshold.sh 0.3   （1.0=実機の最小値 j1-3:10Nm, j4-6:1Nm。小さいほど小さい力で動く。範囲 0.05〜1）
set -e
[ $# -eq 1 ] || { sed -n 2,4p "$0"; exit 1; }
python3 -c "import sys; v=float(sys.argv[1]); sys.exit(0 if 0.05<=v<=1 else '倍率は 0.05〜1')" "$1"
docker exec fr3-real-demo bash -c "source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=58 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST; ros2 param set /link_compliance threshold_scale $1"
