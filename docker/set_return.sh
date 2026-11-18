#!/bin/bash
# 戻る速さを、動かしたまま変える（関節モード・fairino モデル）。
# 使い方: ./set_return.sh 秒    例: ./set_return.sh 2.0   （戻りの時定数。大きいほどゆっくり。既定 0.7、範囲 0.1〜10）
set -e
[ $# -eq 1 ] || { sed -n 2,4p "$0"; exit 1; }
python3 -c "import sys; v=float(sys.argv[1]); sys.exit(0 if 0.1<=v<=10 else 'return_time は 0.1〜10 秒')" "$1"
docker exec fr3-real-demo bash -c "source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=58 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST; ros2 param set /link_compliance return_time $1"
