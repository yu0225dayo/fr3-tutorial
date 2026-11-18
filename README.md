# fr3-real — FR3 アーム実機の作業場

Fairino FR3 の実機を動かす GUI(接続・インピーダンス制御・目的地への移動・禁止エリア)と、シミュレーションでの練習環境。
仮想環境での訓練(Isaac Lab)とは、別の場所で行う。作業のルール・安全の約束は [CLAUDE.md](CLAUDE.md)。

## 動作の様子(動画)
実機を動かしている映像は YouTube にある。画像をクリックすると再生ページに飛ぶ。

[![FR3 実機の動作の様子](https://img.youtube.com/vi/VWkoaYF302Q/hqdefault.jpg)](https://youtu.be/VWkoaYF302Q)

動画: https://youtu.be/VWkoaYF302Q

## メーカー(Fairino)のファイルについて
ロボットのモデル(URDF・メッシュ)、MoveIt 2 の設定、C++ SDK(libfairino)、API の資料は、Fairino が公開している
**[FAIR-INNOVATION/frcobot_ros2](https://github.com/FAIR-INNOVATION/frcobot_ros2)** のものを使う。
ライセンスが示されていないため、このリポジトリには含めていない。`scripts/setup_vendor.sh` が公式から取ってきて置く。
このリポジトリにあるのは、作者が書いたもの(GUI・ROS パッケージ・Docker 環境・設定)と、メーカーの MoveIt 設定への変更([`patches/`](patches/))だけ。
何をどこから取ってくるかは [docs/setup.md](docs/setup.md#メーカーfairinoの公開物について)。

## すぐ始める
```
git clone <このリポジトリの URL>  &&  cd <リポジトリのフォルダ>
scripts/setup_vendor.sh          # メーカーの公開物を取得して配置(git が必要、約 110MB)
gui/build_bridge.sh              # 実機との橋渡し fr3_bridge をビルド(g++ が必要)
./docker/run.sh                  # 初回は Docker イメージを自動でビルド(20〜40 分・約 6GB)。中で:
                                 #   cd /work/ros_ws && colcon build --symlink-install && exit
./docker/run.sh gui              # GUI + RViz(ドライラン)
```
ブラウザで `http://<IP>:6080/vnc.html?autoconnect=true&resize=scale` を開く。
必要なもの・各手順の説明・実機につなぐ前の確認は **[docs/setup.md](docs/setup.md)**。

## 説明
- [docs/setup.md](docs/setup.md): 環境構築(メーカーの公開物の取得、ビルド、起動、実機につなぐ前の確認)
- [docs/gui_usage.md](docs/gui_usage.md): 操作 GUI の使い方と、各タブ・各項目の意味
- [docs/impedance_notes.md](docs/impedance_notes.md): インピーダンス制御(コンプラ制御)の試しで分かったこと(軌道があるときだけ効く、Web のプログラムの書き方と注意)
- [docs/changelog.md](docs/changelog.md): 変更点(日付ごと)

## 実機の安全
- 既定はドライラン(実機には何も送らず、動作の内容だけを表示)。動作を送るのは、`--execute` で起動し、画面の安全確認のチェックと、確認の画面を通したときだけ。
- 関節範囲・速度の上限・手先の作業範囲(`config/limits.json`)と禁止エリア(`config/zones.json`)は、GUI 側でも確認する。使う前に、自分の実機の周囲に合わせること。
- 送ったもの・断ったものは `logs/` に残る。
- 実機を動かすときは、非常停止の場所と、周囲に人・物がないことを必ず確かめる。**使用は自己責任で。**

## 構成
| 場所 | 内容 |
|---|---|
| `gui/` | 操作 GUI(`fr3_core.py` 中核、`fr3_tk.py` Tk 版、`fr3_gui.py` ブラウザ版、`fr3_bridge.cpp` 実機との橋渡し)。`lib/` は setup_vendor.sh が作る |
| `config/` | 速度・関節・TCP 範囲の制限(`limits.json`)、禁止エリア(`zones.json`)、初期姿勢(`init_pose.csv`) |
| `docker/` | ROS 2 Jazzy + MoveIt 2 + RViz + noVNC の環境と起動スクリプト |
| `ros_ws/` | ROS 2 パッケージ(`fr3_compliance`)。メーカーのパッケージは setup_vendor.sh が置く |
| `patches/` | メーカーの MoveIt 設定への変更(初期姿勢・加速度上限・速度の倍率) |
| `scripts/` | メーカーの公開物の取得(`setup_vendor.sh`)、Isaac シミュレーション用の操作盤 |
| `docs/` | 説明 |
| `logs/` | 実行のログ(Git には含めない) |
| `vendor/` | メーカーの公開物(setup_vendor.sh が取得。Git には含めない) |

## ライセンス
このリポジトリのコード・文書は [MIT License](LICENSE)。
`scripts/setup_vendor.sh` が取ってくるメーカー(Fairino)のファイル(`vendor/`、`gui/lib/`、`ros_ws/src/fairino3_v6_moveit2_config/`)は対象外で、その扱いはメーカーに従う。
