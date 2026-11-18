#!/bin/bash
# ランダムデモの開始・停止（関節モードで起動しているとき）。
# 使い方: ./random_demo.sh on          初期姿勢のまま、押す位置・向き・大きさをランダムに選んで、押し続ける
#         ./random_demo.sh on pose     試行ごとに、腕の姿勢もランダムに選ぶ
#         ./random_demo.sh off         止める（初期姿勢へ戻る）
# 画面の上に、段階が文字で出る: HOME（初期姿勢）/ PUSH（力をかけている）/ RETURNING（戻っている）
# 姿勢固定のときは、同じ位置・向きで「小さい力（しきい値未満で動かない）→ 大きい力（動く）」の順に押す。
# 力をかける前に止まる時間: ros2 param set /link_compliance delay_before_push 秒（既定 2）
# 試行は logs/random_trials.jsonl に残る。
set -e
P=false
case "$1" in on) V=true; [ "$2" = pose ] && P=true;; off) V=false;; *) sed -n 2,8p "$0"; exit 1;; esac
docker exec fr3-real-demo bash -c "source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=58 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST; ros2 param set /link_compliance random_pose $P; ros2 param set /link_compliance random_demo $V"
