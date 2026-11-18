#!/bin/bash
# コンテナを起動する。ROS の通信は自分のマシン内に閉じる（同じ LAN の他の ROS と混ざらない）。
# 使い方: ./run.sh                    対話シェル
#         ./run.sh demo               RViz + MoveIt（偽コントローラ）を起動。ブラウザで画面を見る
#         ./run.sh compliance         コンプライアンス制御のデモ(demo とは同時不可)
#         ./run.sh compliance-joint   関節モード（リンクの途中に力をかける。push_link.sh）
#             MODEL=spring ./run.sh compliance-joint   しきい値なしの単純なばねにする（既定は実機のインピーダンス制御の再現）
#         ./run.sh gui [--execute]     操作 GUI(Tk)+ RViz を並べて表示。実機へは既定でドライラン
#         ./run.sh stop               デモを止める
#         NOVNC_PORT=6081 ./run.sh demo   画面のポートを変える
HERE="$(cd "$(dirname "$0")" && pwd)"
WS="$(cd "$HERE/.." && pwd)"
VENDOR="$(readlink -f "$WS/docs/vendor_frcobot_ros2")"
COMMON=(--rm --network host --user "$(id -u):$(id -g)" -e HOME=/tmp/home
  -e NOVNC_PORT="${NOVNC_PORT:-6080}" -e VNC_DISPLAY="${VNC_DISPLAY:-:51}" -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-58}"
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  -v "$WS":/work -v "$VENDOR":/vendor:ro)
# イメージ fr3-real:jazzy が無い PC(取得直後など)では、Dockerfile から自動でビルドする(初回のみ。インターネットが必要で、20〜40 分ほど)
if [ "$1" != stop ] && ! docker image inspect fr3-real:jazzy >/dev/null 2>&1; then
  echo "[run.sh] イメージ fr3-real:jazzy が無いので、ビルドします(初回のみ)..."
  docker build -t fr3-real:jazzy "$HERE" || { echo "[run.sh] ビルドに失敗しました"; exit 1; }
fi
SRC='mkdir -p $HOME && source /opt/ros/jazzy/setup.bash && source /work/ros_ws/install/setup.bash 2>/dev/null'
case "$1" in
  demo)
    exec docker run -d "${COMMON[@]}" --name fr3-real-demo fr3-real:jazzy bash -c "$SRC; /work/docker/start_gui.sh || exit 1; export DISPLAY=${VNC_DISPLAY:-:51} LIBGL_ALWAYS_SOFTWARE=1; exec ros2 launch fairino3_v6_moveit2_config demo.launch.py" ;;
  compliance|compliance-joint)
    MODE=wrist; [ "$1" = compliance-joint ] && MODE=joint
    # コンプライアンス制御のデモ（偽の力を流して、RViz で、押されてずれて戻る動きを見る）。demo とは同時に動かせない
    docker stop fr3-real-demo >/dev/null 2>&1
    exec docker run -d "${COMMON[@]}" --name fr3-real-demo fr3-real:jazzy bash -c "$SRC; /work/docker/start_gui.sh || exit 1; export DISPLAY=${VNC_DISPLAY:-:51} LIBGL_ALWAYS_SOFTWARE=1; exec ros2 launch fr3_compliance compliance.launch.py mode:=$MODE model:=${MODEL:-fairino}" ;;
  gui)
    # 操作 GUI。RViz(左)と Tk の GUI(右)を noVNC に並べる。シミュレーション/実機は GUI 内で選ぶ。日本語表示のためホストのフォントを借りる
    shift; docker stop fr3-real-demo >/dev/null 2>&1
    exec docker run -d "${COMMON[@]}" -e GUI_LAYOUT=split -v /usr/share/fonts:/usr/share/fonts:ro --name fr3-real-demo fr3-real:jazzy bash -c "$SRC; /work/docker/start_gui.sh || exit 1; export DISPLAY=${VNC_DISPLAY:-:51} LIBGL_ALWAYS_SOFTWARE=1; ros2 launch /work/gui/gui_view.launch.py >/tmp/view.log 2>&1 & sleep 4; cd /work/gui && exec python3 fr3_tk.py $*" ;;
  stop)
    docker stop fr3-real-demo ;;
  *)
    exec docker run -it "${COMMON[@]}" --name fr3-real-ros fr3-real:jazzy bash -c "$SRC; exec bash" ;;
esac
