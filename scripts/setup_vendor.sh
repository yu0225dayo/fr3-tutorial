#!/bin/bash
# メーカー(Fairino)の公開物を取ってきて、このリポジトリで使う場所に置く。取得したあと 1 回実行する(インターネットと git が必要)。
#
# メーカーのファイル(URDF・メッシュ・MoveIt 設定・SDK のヘッダと libfairino・API の PDF)は、ライセンスが示されていないため、
# このリポジトリには含めていない。公式の https://github.com/FAIR-INNOVATION/frcobot_ros2 から取ってくる。
#
# 作るもの(どれも .gitignore 済み):
#   vendor/frcobot_ros2/                         公式リポジトリ(使う部分だけを取り出した sparse checkout)
#   gui/lib/libfairino.so*                       C++ SDK 2.3.7(fairino_hardware_v3_9_7 にあるもの)
#   ros_ws/src/fairino3_v6_moveit2_config/       公式の MoveIt 設定に、patches/ の変更(初期姿勢・加速度上限・速度の倍率)を当てたもの
#   docs/vendor_frcobot_ros2                     vendor/frcobot_ros2 へのリンク(docker/run.sh が /vendor として渡す)
#
# 使い方: scripts/setup_vendor.sh            取得・配置(済んでいる物は飛ばす)
#         scripts/setup_vendor.sh --force    配置し直す(gui/lib と MoveIt 設定を作り直す)
set -euo pipefail
cd "$(dirname "$0")/.."

REPO=https://github.com/FAIR-INNOVATION/frcobot_ros2
# 動作を確かめた公式のコミット(2026-10-30)。新しい版を試すときは、ここを書き換えて --force で配置し直す
REV=eb6d7ce40071aeb1a0c06e40e95f3be46bd7594e
HW=fairino_hardware_v3_9_7            # SDK の版: libfairino 2.3.7(コントローラ V3.9.7 向け)
SDK_SO=libfairino.so.2.3.7
MOVEIT=fairino3_v6_moveit2_config
DEST=vendor/frcobot_ros2
FORCE=0; [ "${1:-}" = --force ] && FORCE=1

command -v git >/dev/null || { echo "git が必要です(sudo apt install git)"; exit 1; }

# 1. 公式リポジトリを取ってくる。全体は 100MB を超えるので、使う部分だけを取り出す
if [ ! -d "$DEST/.git" ]; then
  echo "[1/4] $REPO を取得します..."
  mkdir -p vendor
  git clone --filter=blob:none --no-checkout "$REPO" "$DEST"
fi
git -C "$DEST" sparse-checkout set fairino_description fairino_msgs "$MOVEIT" "$HW/libfairino"
if [ "$(git -C "$DEST" rev-parse HEAD 2>/dev/null)" != "$REV" ] || [ -z "$(ls -A "$DEST" | grep -v '^\.git$')" ]; then
  git -C "$DEST" fetch -q origin "$REV" 2>/dev/null || true
  git -C "$DEST" -c advice.detachedHead=false checkout -q "$REV"
fi
echo "[1/4] OK: $DEST($(git -C "$DEST" rev-parse --short HEAD))"

# 2. SDK(libfairino)。公式リポジトリでは libfairino.so / .so.2 がリンクではなくファイル名だけの文字列なので、ここでリンクを作る
if [ $FORCE = 1 ] || [ ! -f "gui/lib/$SDK_SO" ]; then
  mkdir -p gui/lib
  cp "$DEST/$HW/libfairino/lib/$SDK_SO" gui/lib/
  ln -sf "$SDK_SO" gui/lib/libfairino.so.2
  ln -sf libfairino.so.2 gui/lib/libfairino.so
fi
echo "[2/4] OK: gui/lib/$SDK_SO"

# 3. MoveIt 設定 = 公式のもの + patches/ の変更
if [ $FORCE = 1 ] || [ ! -d "ros_ws/src/$MOVEIT" ]; then
  rm -rf "ros_ws/src/$MOVEIT"
  cp -r "$DEST/$MOVEIT" "ros_ws/src/$MOVEIT"
  patch -s -p1 -d "ros_ws/src/$MOVEIT" < "patches/$MOVEIT.patch"
fi
echo "[3/4] OK: ros_ws/src/$MOVEIT(patches/$MOVEIT.patch を適用)"

# 4. docker/run.sh が使うリンク
[ -e docs/vendor_frcobot_ros2 ] || ln -s ../vendor/frcobot_ros2 docs/vendor_frcobot_ros2
echo "[4/4] OK: docs/vendor_frcobot_ros2 -> $(readlink docs/vendor_frcobot_ros2)"
echo "完了。次は gui/build_bridge.sh(実機との橋渡しのビルド)。"
