#!/bin/bash
# リンクの真ん中に力をかける（関節モード: ./run.sh compliance-joint で起動しているとき）。終わると 0 に戻す。
# 使い方: ./push_link.sh 区間 FX FY FZ 秒 [位置]   例: ./push_link.sh 3 15 0 0 4 0.8   （前腕の先端寄り(0.8)に x 方向 15N を 4 秒）
#   区間 s = 関節 j_s と j_(s+1) の間のリンク（1〜5。3 = 前腕）。6 = 手首の先。位置 0=根元側〜1=先端側（省略 0.5）
#   力は base_link 座標系 [N]。上限 50N
set -e
[ $# -eq 5 ] || [ $# -eq 6 ] || { sed -n 2,6p "$0"; exit 1; }
python3 - "$@" <<'PY'
import sys
s=int(sys.argv[1]); v=[float(x) for x in sys.argv[2:5]]
if not 1<=s<=6: sys.exit("区間は 1〜6")
if len(sys.argv)>6 and not 0<=float(sys.argv[6])<=1: sys.exit("位置は 0〜1")
if any(abs(x)>50 for x in v): sys.exit("上限を超えています（50N まで）")
PY
SEC=$5; POS=${6:-0.5}
docker exec fr3-real-demo bash -c "
source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=58 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
timeout $SEC ros2 topic pub -r 20 /link_force std_msgs/msg/Float64MultiArray '{data: [$1,$2,$3,$4,$POS]}' >/dev/null 2>&1 || true
ros2 topic pub --once /link_force std_msgs/msg/Float64MultiArray '{data: [$1,0,0,0,$POS]}' >/dev/null 2>&1
echo '力をかけ終わりました（0 に戻しました）'"
