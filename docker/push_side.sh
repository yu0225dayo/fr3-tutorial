#!/bin/bash
# 真横から力をかける（関節モード）。真横 = 腕が動く平面（j2 の軸）に垂直な水平方向。腕が回っても真横のまま。終わると 0 に戻す。
# 使い方: ./push_side.sh 区間 力[N] 秒 [位置] [+|-]   例: ./push_side.sh 6 40 4        （手首の先を、真横に 40N、4 秒）
#                                                       ./push_side.sh 5 40 4 0.8 -    （反対側から）
#   区間 1〜6（6 = 手首の先）、位置 0〜1（省略 0.5）、+ と - は左右（省略 +）。力の上限 50N
set -e
[ $# -ge 3 ] && [ $# -le 5 ] || { sed -n 2,6p "$0"; exit 1; }
python3 - "$@" <<'PY'
import sys
s=int(sys.argv[1]); f=float(sys.argv[2])
if not 1<=s<=6: sys.exit("区間は 1〜6")
if not 0<f<=50: sys.exit("力は 0 より大きく 50N まで")
if len(sys.argv)>4 and not 0<=float(sys.argv[4])<=1: sys.exit("位置は 0〜1")
if len(sys.argv)>5 and sys.argv[5] not in "+-": sys.exit("向きは + か -")
PY
SEG=$1; F=$2; SEC=$3; POS=${4:-0.5}; SGN=${5:-+}; [ "$SGN" = "-" ] && F=-$F
docker exec fr3-real-demo bash -c "
source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=58 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
timeout $SEC ros2 topic pub -r 20 /link_force std_msgs/msg/Float64MultiArray '{data: [$SEG,$F,0,0,$POS,1]}' >/dev/null 2>&1 || true
ros2 topic pub --once /link_force std_msgs/msg/Float64MultiArray '{data: [$SEG,0,0,0,$POS,1]}' >/dev/null 2>&1
echo '力をかけ終わりました（0 に戻しました）'"
