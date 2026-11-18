#!/bin/bash
# 画面の見え方: ブラウザ側で「Scaling mode = Local scaling」(URL に ?resize=scale を付ける)。
# 仮想画面(Xvnc) + noVNC を起動する。ブラウザで http://<サーバIP>:${NOVNC_PORT:-6080}/vnc.html を開く。
# コンテナ内に自分のユーザー名が無くても動くよう、vncserver ラッパーは使わず Xvnc を直接起動する。
# --network host では X の番号がホストと共有になる(ホストの画面が :0 / :1 だと、ぶつかってホストの画面に出てしまう)。
# そのため、ホストで使わない番号(既定 :51)を使う。VNC のポートは 5900+番号
VNC_DISPLAY="${VNC_DISPLAY:-:51}"
export DISPLAY="$VNC_DISPLAY"
RFB_PORT=$((5900 + ${VNC_DISPLAY#:}))
Xvnc "$DISPLAY" -geometry 1600x900 -depth 24 -rfbport "$RFB_PORT" -localhost -SecurityTypes None >/tmp/vnc.log 2>&1 &
XVNC_PID=$!
for i in $(seq 1 20); do xdpyinfo -display "$DISPLAY" >/dev/null 2>&1 && break; sleep 0.5; done
# Xvnc が起動できなかったら(番号の衝突など)止める。ホストの画面に出てしまうのを防ぐ
if ! kill -0 "$XVNC_PID" 2>/dev/null; then echo "[start_gui] Xvnc を $DISPLAY で起動できません: $(tail -3 /tmp/vnc.log)" >&2; exit 1; fi
# 全ウィンドウを最大化して開く（RViz が画面の一部しか使わず、縦横比が崩れて見えるのを防ぐ）
mkdir -p "$HOME/.config/openbox"
if [ "$GUI_LAYOUT" = split ]; then
# 左 1000px に RViz、右 600px に操作 GUI(タイトル FR3 Control)を並べる
cat > "$HOME/.config/openbox/rc.xml" <<'XML'
<?xml version="1.0" encoding="UTF-8"?>
<openbox_config xmlns="http://openbox.org/3.4/rc">
  <applications>
    <application title="FR3 Control"><decor>no</decor><position force="yes"><x>1000</x><y>0</y></position><size><width>600</width><height>900</height></size></application>
    <application class="rviz2"><decor>no</decor><position force="yes"><x>0</x><y>0</y></position><size><width>1000</width><height>900</height></size></application>
  </applications>
</openbox_config>
XML
else
cat > "$HOME/.config/openbox/rc.xml" <<'XML'
<?xml version="1.0" encoding="UTF-8"?>
<openbox_config xmlns="http://openbox.org/3.4/rc">
  <applications><application class="*"><maximized>yes</maximized><decor>no</decor></application></applications>
</openbox_config>
XML
fi
openbox >/tmp/openbox.log 2>&1 &
websockify --web /usr/share/novnc "${NOVNC_PORT:-6080}" "localhost:$RFB_PORT" >/tmp/novnc.log 2>&1 &
echo "noVNC: http://<サーバIP>:${NOVNC_PORT:-6080}/vnc.html （認証なし・LAN 内のみ）"
