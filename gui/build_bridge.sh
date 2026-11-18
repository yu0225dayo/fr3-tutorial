#!/bin/bash
# 実機との橋渡し fr3_bridge をビルドする(g++ が必要)。libfairino は gui/lib/ のもの(2.3.7)を使う。
# 先に scripts/setup_vendor.sh を実行しておく(SDK のヘッダと gui/lib/ を公式から取ってくる)。
cd "$(dirname "$0")"
[ -f lib/libfairino.so.2.3.7 ] || { echo "gui/lib/ に libfairino がありません。先に scripts/setup_vendor.sh を実行してください"; exit 1; }
INC=../vendor/frcobot_ros2/fairino_hardware_v3_9_7/libfairino/include
g++ -std=c++14 -O2 fr3_bridge.cpp -I"$INC" -Llib -lfairino -Wl,-rpath,'$ORIGIN/lib' -lpthread -o fr3_bridge && echo "OK: $(pwd)/fr3_bridge"
