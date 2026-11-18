"""FR3 操作 GUI(ブラウザ版)。python3 fr3_gui.py [--execute] [--host 0.0.0.0]。禁止エリアの編集は Tk 版(fr3_tk.py)で行う。"""
import argparse
import json
import os

from flask import Flask, jsonify, request

from fr3_core import *  # noqa
from fr3_core import HERE, J_MIN, J_MAX, HOME_DEG, LIM


def create_app(execute):
    A = App(execute)
    web = Flask(__name__)

    @web.get("/")
    def index():
        return open(os.path.join(HERE, "index.html"), encoding="utf-8").read()

    @web.get("/api/info")
    def info():
        return jsonify(limits=LIM, execute=execute, home=HOME_DEG, j_min=J_MIN.tolist(), j_max=J_MAX.tolist())

    @web.get("/api/state")
    def state():
        s = A.be.state() if A.be.connected else {}
        return jsonify(backend=A.be.name, connected=A.be.connected, dry=A.dry(), impedance=A.impedance_on, safety=A.safety_ok, **s)

    @web.post("/api/connect")
    def connect():
        d = request.get_json()
        if A.be.connected:
            A.be.disconnect()
        A.impedance_on = False
        A.be = A.real if d["backend"] == "real" else A.sim
        A.safety_ok = False
        r = A.be.connect(d.get("ip", ""))
        log(action="connect", backend=A.be.name, ip=d.get("ip"), result=r)
        return jsonify(r)

    @web.post("/api/disconnect")
    def disconnect():
        if A.impedance_on:
            A.imp({"on": False, "ws": 0})
        r = A.be.disconnect()
        log(action="disconnect")
        return jsonify(r)

    @web.post("/api/safety")
    def safety():
        A.safety_ok = bool(request.get_json()["ok"])
        log(action="safety_check", ok=A.safety_ok)
        return jsonify(ok=True)

    @web.post("/api/move")
    def move():
        return jsonify(A.move(request.get_json()))

    @web.post("/api/stop")
    def stop():  # 停止は常に通す(検査なし)
        r = A.be.stop() if A.be.connected else {"ok": False, "msg": "未接続"}
        log(action="stop", result=r)
        return jsonify(r)

    @web.post("/api/impedance")
    def impedance():
        return jsonify(A.imp(request.get_json()))

    @web.post("/api/enable")
    def enable():
        d = request.get_json()
        if not A.be.connected or A.dry() or (A.be.sends_to_robot and not A.safety_ok):
            return jsonify(ok=False, msg="接続済みで、実行モードかつ安全確認済みのときだけ")
        r = A.be.enable(d["on"])
        log(action="enable", on=d["on"], result=r)
        return jsonify(r)

    @web.post("/api/reset")
    def reset():
        r = A.be.reset() if A.be.connected and not A.dry() else {"ok": False, "msg": "実行モードで接続済みのときだけ"}
        log(action="reset_error", result=r)
        return jsonify(r)

    return web


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8096)
    ap.add_argument("--execute", action="store_true", help="実機に動作を送る(既定はドライラン)")
    a = ap.parse_args()
    print(f"[gui] http://{a.host}:{a.port}/  実機への送信: {'有効(--execute)' if a.execute else '無効(ドライラン)'}", flush=True)
    create_app(a.execute).run(host=a.host, port=a.port, threaded=True)
