# 環境構築

このリポジトリを取得してから、GUI でシミュレーション・実機を動かせるようになるまでの手順。
使い方は [gui_usage.md](gui_usage.md)、安全の約束は [CLAUDE.md](../CLAUDE.md)。

## メーカー(Fairino)の公開物について
URDF・メッシュ・MoveIt 設定・C++ SDK(libfairino)・API の資料は、Fairino が GitHub で公開しているものを使う。
これらにはライセンスが示されていないため、**このリポジトリには含めず**、`scripts/setup_vendor.sh` が公式から取ってくる。

| 使うもの | 公式の場所 | 置く場所 |
|---|---|---|
| ロボットのモデル(URDF・メッシュ) | [frcobot_ros2 / fairino_description](https://github.com/FAIR-INNOVATION/frcobot_ros2/tree/main/fairino_description) | `vendor/frcobot_ros2/fairino_description`(ROS では `ros_ws/src/fairino_description` から参照) |
| ROS 2 のメッセージ定義 | [frcobot_ros2 / fairino_msgs](https://github.com/FAIR-INNOVATION/frcobot_ros2/tree/main/fairino_msgs) | `vendor/frcobot_ros2/fairino_msgs` |
| MoveIt 2 の設定(FR3) | [frcobot_ros2 / fairino3_v6_moveit2_config](https://github.com/FAIR-INNOVATION/frcobot_ros2/tree/main/fairino3_v6_moveit2_config) | `ros_ws/src/fairino3_v6_moveit2_config`(下の変更を当てる) |
| C++ SDK(libfairino 2.3.7、ヘッダ) | [frcobot_ros2 / fairino_hardware_v3_9_7 / libfairino](https://github.com/FAIR-INNOVATION/frcobot_ros2/tree/main/fairino_hardware_v3_9_7/libfairino) | `gui/lib/`(ライブラリ)、ヘッダは `vendor/` から直接 |
| API・エラーコードの資料(PDF) | [frcobot_ros2](https://github.com/FAIR-INNOVATION/frcobot_ros2) の直下 | `vendor/frcobot_ros2/`(`docs/vendor_frcobot_ros2` からも見える) |

- 取ってくる版は、動作を確かめたコミット `eb6d7ce`(2026-10-30)に固定している(`scripts/setup_vendor.sh` の `REV`)。
- MoveIt 設定への変更は [`patches/fairino3_v6_moveit2_config.patch`](../patches/fairino3_v6_moveit2_config.patch) にまとめてある。
  - `initial_positions.yaml`: 初期姿勢を、実機の初期姿勢(`config/init_pose.csv`)にする(元は全関節 0)
  - `joint_limits.yaml`: 全関節に加速度の上限 3.0 rad/s² を入れる(元は上限なしで、時間配分が失敗するため。仮の値)、速度・加速度の倍率を 0.05 にする
  - `moveit.rviz`: 軌道のループ表示を切り、倍率を 0.05 にする
- メーカーの説明書: [Fairino のドキュメント(ROS ガイド)](https://fair-documentation.readthedocs.io/en/latest/ROSGuide/index.html)、C++ SDK 単体は [fairino-cpp-sdk](https://github.com/FAIR-INNOVATION/fairino-cpp-sdk)。

## 必要なもの
- Ubuntu(22.04 / 24.04 で想定。ROS 2 は Docker の中で動かすので、ホストに入れなくてよい)
- `git`、`g++`、`patch`(`sudo apt install git g++ patch`)
- Docker(RViz と並べた GUI を使うとき。ユーザーが `docker` を sudo なしで使えること)
- インターネット(メーカーの公開物の取得と、Docker イメージの初回ビルド)

## 手順
### 1. 取得と、メーカーの公開物の配置
```
git clone <このリポジトリの URL>
cd <リポジトリのフォルダ>
scripts/setup_vendor.sh      # 公式の frcobot_ros2 を vendor/ に取得し、gui/lib/ と MoveIt 設定を置く(約 110MB)
```
もう一度実行しても、済んでいる物は飛ばす。配置し直すときは `scripts/setup_vendor.sh --force`。

### 2. 実機との橋渡し(fr3_bridge)のビルド
```
gui/build_bridge.sh          # gui/fr3_bridge ができる(libfairino を gui/lib/ から読む)
```
GUI は、実機との通信をこの `fr3_bridge` に任せる。シミュレーションだけなら、無くても起動できる。

### 3. Docker イメージと ROS 2 ワークスペース
```
./docker/run.sh              # 対話シェル。イメージ fr3-real:jazzy が無ければ、docker/Dockerfile から自動でビルドする
                             # (初回のみ。20〜40 分、約 6GB)
# ↓ コンテナの中で
cd /work/ros_ws && colcon build --symlink-install
exit
```
- イメージは ROS 2 Jazzy + MoveIt 2 + RViz + noVNC(ブラウザで画面を見る)。方策用の Python 3.11 環境(`/opt/policy-venv`)も入っている。
- `ros_ws/src/fairino_description` と `fairino_msgs` は、コンテナの中の `/vendor/...` へのリンク。`docker/run.sh` が `vendor/frcobot_ros2` を `/vendor` として渡すので、ビルドはコンテナの中で行う。
- ビルドの成果物(`ros_ws/build`、`install`、`log`)は、ホスト側のこのフォルダに残る。

### 4. 起動
```
./docker/run.sh gui              # GUI + RViz。実機へは送らない(ドライラン)
./docker/run.sh gui --execute    # 実機に送れる状態(送るたびに確認の画面が出る)
./docker/run.sh stop
```
ブラウザで `http://<この PC の IP>:6080/vnc.html?autoconnect=true&resize=scale` を開く。
ほかの起動のしかた(MoveIt のデモ、コンプライアンス制御のデモ)は `docker/run.sh` の先頭に書いてある。

ブラウザ版の簡易 GUI は、Docker なしでも動く: `cd gui && python3 fr3_gui.py`(`pip install flask numpy` が必要)。

## 実機につなぐ前に
1. PC を、実機と同じネットワーク(192.168.58.x)に入れる。コントローラの出荷時の IP は 192.168.58.2。
2. 読み取りだけで接続し、コントローラの版を確認する。SDK は libfairino 2.3.7(コントローラ V3.9.7 向け)を使っている。版が違うときは、`scripts/setup_vendor.sh` の `HW`・`SDK_SO` を合う版(`fairino_hardware_v*`)に変えて `--force` で配置し直し、`gui/build_bridge.sh` の版の表記も合わせる。
3. `config/limits.json`(関節範囲・速度の上限・手先の作業範囲)と `config/zones.json`(禁止エリア)を、実機の周囲に合わせる。初期姿勢は `config/init_pose.csv`。
4. 既定はドライラン。動作を送るのは、`--execute` で起動し、画面の安全確認のチェックと、確認の画面を通したときだけ。

## 補足
- `scripts/sim_controller.py` と `scripts/sim_push.py` は、Isaac Lab(別の環境)で動かすシミュレーション用。起動のしかたは各ファイルの先頭に書いてある。
