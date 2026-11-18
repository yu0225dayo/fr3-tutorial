#!/bin/bash
# 偽の力覚センサに、力を一定時間かける（コンプライアンスのデモ用）。終わると 0 に戻す。
# 使い方: ./push.sh FX FY FZ TX TY TZ 秒    例: ./push.sh 0 0 20 0 0 0 4   （手先の座標系で z 方向に 20N を 4 秒）
#   力 [N]、トルク [Nm]。上限: 力 50N、トルク 10Nm（安全のため、超える値は拒否する）
set -e
[ $# -eq 7 ] || { sed -n 2,5p "$0"; exit 1; }
python3 - "$@" <<'PY'
import sys
v=[float(x) for x in sys.argv[1:7]]
if any(abs(x)>50 for x in v[:3]) or any(abs(x)>10 for x in v[3:]):
    sys.exit("上限を超えています（力 50N、トルク 10Nm まで）")
PY
SEC=$7
docker exec fr3-real-demo bash -c "
source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=58 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
timeout $SEC ros2 topic pub -r 20 /fake_ft_command/commands std_msgs/msg/Float64MultiArray '{data: [$1,$2,$3,$4,$5,$6]}' >/dev/null 2>&1 || true
ros2 topic pub --once /fake_ft_command/commands std_msgs/msg/Float64MultiArray '{data: [0,0,0,0,0,0]}' >/dev/null 2>&1
echo '力をかけ終わりました（0 に戻しました）'"
