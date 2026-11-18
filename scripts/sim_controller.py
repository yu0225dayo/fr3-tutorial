"""FR3 のシミュレーション用「操作盤」。ブラウザのボタンでアーム 6 関節を動かす。

実機の流れ(読み取り → 低速 → 関節制限の確認 → ログ)に合わせた練習用。シミュレーションなので、
実機には何も送らない。ハンドの指は 0 のまま固定。

起動例(conda isaaclab、GPU は CUDA_VISIBLE_DEVICES で明示):
    cd <IsaacLab のフォルダ>
    CUDA_VISIBLE_DEVICES=1 python <このリポジトリ>/scripts/sim_controller.py --headless --enable_cameras

ブラウザで http://<サーバIP>:8095/ を開く(認証なし・LAN 内のみ)。
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--port", type=int, default=8095)
parser.add_argument("--speed", type=float, default=15.0, help="関節の最大速度の初期値 [deg/s]")
parser.add_argument("--max_speed", type=float, default=45.0, help="速度の上限 [deg/s]")
parser.add_argument("--duration", type=float, default=0.0, help="秒。0 なら止めるまで動かす(テスト用)")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import io
import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import torch
from PIL import Image

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.sensors import Camera, CameraCfg
from isaaclab.sim import SimulationContext
from isaaclab_assets.robots.fairino_fr3_hx5d20 import FAIRINO_FR3_HX5D20_RIGHT_CFG

LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "logs")  # このリポジトリの logs/
JOINTS = ["j1", "j2", "j3", "j4", "j5", "j6"]
LIMIT_MARGIN_DEG = 5.0  # USD の関節範囲から、さらに内側に取る余裕
NUM_ARM = 6

# 操作盤と物理ループの共有状態
lock = threading.Lock()
shared = {
    "goal": None,  # 目標(rad)。ボタンで変わる
    "cmd": None,  # 実際に PD へ渡している目標(rad)。速度制限つきで goal に近づく
    "pos": [0.0] * NUM_ARM,  # 実際の関節角(rad)
    "speed": args_cli.speed,  # deg/s
    "jpeg": b"",
    "limits": None,  # [(lo, hi)] rad、余裕込み
    "home": None,
}
os.makedirs(LOG_DIR, exist_ok=True)
log_path = os.path.join(LOG_DIR, time.strftime("sim_controller_%Y%m%d_%H%M%S.jsonl"))
log_f = open(log_path, "a")


def log(entry):
    entry["t"] = time.time()
    log_f.write(json.dumps(entry) + "\n")
    log_f.flush()


def clamp_goal(goal):
    lim = shared["limits"]
    return [min(max(g, lo), hi) for g, (lo, hi) in zip(goal, lim)]


def handle_cmd(c):
    """ボタンの指令を goal に反映する。範囲外は、制限の内側に切り詰める。"""
    kind = c.get("type")
    with lock:
        if kind == "jog":
            i = JOINTS.index(c["joint"])
            goal = list(shared["goal"])
            goal[i] += math.radians(float(c["delta_deg"]))
            shared["goal"] = clamp_goal(goal)
        elif kind == "set":
            shared["goal"] = clamp_goal([math.radians(float(v)) for v in c["joints_deg"]])
        elif kind == "home":
            shared["goal"] = list(shared["home"])
        elif kind == "stop":
            shared["goal"] = list(shared["cmd"])
        elif kind == "speed":
            shared["speed"] = min(max(float(c["deg_s"]), 1.0), args_cli.max_speed)
        else:
            return False
        log({"cmd": c, "goal_deg": [round(math.degrees(g), 2) for g in shared["goal"]], "speed": shared["speed"]})
    return True


PAGE = """<!doctype html><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>FR3 操作盤(シミュレーション)</title>
<style>
body{font-family:sans-serif;margin:12px;background:#fafafa;color:#222}
.wrap{display:flex;flex-wrap:wrap;gap:16px}
img{width:640px;max-width:100%;border:1px solid #ccc;background:#000}
table{border-collapse:collapse}td,th{padding:3px 6px;text-align:right}
button{font-size:15px;padding:6px 10px;margin:1px}
.stop{background:#d33;color:#fff;font-weight:bold;font-size:18px;padding:10px 24px}
.warn{color:#a50}
</style>
<h2>FR3 操作盤 <small class=warn>シミュレーション(実機には送られません)</small></h2>
<div class=wrap>
<div><img id=cam src=/frame.jpg></div>
<div>
<table id=tbl><tr><th>関節<th>現在[deg]<th>目標[deg]<th>範囲<th colspan=4>動かす[deg]</tr></table>
<p>歩幅: <select id=step><option>1<option selected>5<option>10</select> deg
　速度: <input id=spd type=range min=1 max=__MAXSPD__ value=__SPD__ style="width:160px"> <span id=spdv></span> deg/s</p>
<p><button onclick="send({type:'home'})">初期姿勢へ</button>
<button class=stop onclick="send({type:'stop'})">停止</button></p>
<p>プリセット: <button onclick="preset([0,-90,90,-90,-90,-100])">姿勢1</button>
<button onclick="preset([-30,-80,70,-70,-90,-100])">姿勢2</button>
<button onclick="preset([20,-100,80,-70,-90,-100])">姿勢3</button></p>
<p class=warn>範囲外は自動で内側に切り詰め、速度は上限つきで滑らかに動きます。操作はすべて logs/ に記録されます。</p>
</div></div>
<script>
const J=['j1','j2','j3','j4','j5','j6'];
const tbl=document.getElementById('tbl');
J.forEach(j=>{const r=tbl.insertRow();r.innerHTML=`<td>${j}<td id=p_${j}><td id=g_${j}><td id=l_${j}>
<td><button onclick="jog('${j}',-1)">−−</button><td><button onclick="jog('${j}',-0.2)">−</button>
<td><button onclick="jog('${j}',0.2)">＋</button><td><button onclick="jog('${j}',1)">＋＋</button>`});
function send(c){return fetch('/cmd',{method:'POST',body:JSON.stringify(c)})}
function jog(j,k){send({type:'jog',joint:j,delta_deg:k*5*document.getElementById('step').value/5})}
function preset(v){send({type:'set',joints_deg:v})}
const spd=document.getElementById('spd');
spd.oninput=()=>{document.getElementById('spdv').textContent=spd.value};spd.onchange=()=>send({type:'speed',deg_s:+spd.value});
document.getElementById('spdv').textContent=spd.value;
async function tick(){try{const s=await (await fetch('/state')).json();
J.forEach((j,i)=>{document.getElementById('p_'+j).textContent=s.pos[i].toFixed(1);
document.getElementById('g_'+j).textContent=s.goal[i].toFixed(1);
document.getElementById('l_'+j).textContent=s.limits[i][0].toFixed(0)+'〜'+s.limits[i][1].toFixed(0)});
}catch(e){}
document.getElementById('cam').src='/frame.jpg?'+Date.now();setTimeout(tick,150)}
tick();
</script>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, ctype, body):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?")[0]
        if path == "/":
            html = PAGE.replace("__MAXSPD__", str(int(args_cli.max_speed))).replace("__SPD__", str(int(shared["speed"])))
            self._send(200, "text/html; charset=utf-8", html.encode())
        elif path == "/frame.jpg":
            self._send(200, "image/jpeg", shared["jpeg"])
        elif path == "/state":
            with lock:
                d = {
                    "pos": [math.degrees(v) for v in shared["pos"]],
                    "goal": [math.degrees(v) for v in shared["goal"]],
                    "speed": shared["speed"],
                    "limits": [[math.degrees(lo), math.degrees(hi)] for lo, hi in shared["limits"]],
                }
            self._send(200, "application/json", json.dumps(d).encode())
        else:
            self._send(404, "text/plain", b"not found")

    def do_POST(self):
        if self.path == "/cmd":
            n = int(self.headers.get("Content-Length", 0))
            try:
                ok = handle_cmd(json.loads(self.rfile.read(n)))
            except Exception as e:  # 不正な指令は無視して記録
                log({"bad_cmd": str(e)})
                ok = False
            self._send(200 if ok else 400, "application/json", b'{"ok":%s}' % (b"true" if ok else b"false"))
        else:
            self._send(404, "text/plain", b"not found")


def main():
    sim = SimulationContext(sim_utils.SimulationCfg(device=args_cli.device, dt=1 / 60))
    sim_utils.GroundPlaneCfg().func("/World/defaultGroundPlane", sim_utils.GroundPlaneCfg())
    light = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.8, 0.8, 0.8))
    light.func("/World/Light", light)

    robot_cfg = FAIRINO_FR3_HX5D20_RIGHT_CFG.copy()
    robot_cfg.prim_path = "/World/Robot"
    robot = Articulation(cfg=robot_cfg)

    eye, lookat = (1.05, 1.05, 0.9), (0.0, 0.0, 0.4)
    sim.set_camera_view(list(eye), list(lookat))
    camera = Camera(
        CameraCfg(
            prim_path="/World/RecordCamera",
            update_period=0,
            height=480,
            width=640,
            data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(focal_length=17.0, focus_distance=400.0, horizontal_aperture=20.955),
        )
    )
    sim.reset()
    dt = sim.get_physics_dt()
    device = robot.data.joint_pos.device

    robot.write_joint_state_to_sim(robot.data.default_joint_pos.clone(), torch.zeros_like(robot.data.default_joint_vel))
    robot.reset()
    camera.set_world_poses_from_view(
        torch.tensor([eye], dtype=torch.float32, device=device), torch.tensor([lookat], dtype=torch.float32, device=device)
    )

    arm_ids = [robot.joint_names.index(j) for j in JOINTS]
    lim = robot.data.joint_pos_limits[0, arm_ids].cpu().numpy()
    m = math.radians(LIMIT_MARGIN_DEG)
    home = [float(v) for v in robot.data.default_joint_pos[0, arm_ids].cpu().numpy()]
    with lock:
        shared["limits"] = [(float(lo) + m, float(hi) - m) for lo, hi in lim]
        shared["home"] = home
        shared["goal"] = list(home)
        shared["cmd"] = list(home)
    print("[ctrl] 関節範囲[deg](余裕込み):", [(round(math.degrees(a)), round(math.degrees(b))) for a, b in shared["limits"]], flush=True)
    print(f"[ctrl] ログ: {log_path}", flush=True)

    server = ThreadingHTTPServer(("0.0.0.0", args_cli.port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"[ctrl] 操作盤: http://10.40.1.115:{args_cli.port}/  (認証なし・LAN 内のみ)", flush=True)

    full_target = robot.data.default_joint_pos.clone()
    step = 0
    t_end = time.time() + args_cli.duration if args_cli.duration > 0 else None
    while simulation_app.is_running() and (t_end is None or time.time() < t_end):
        with lock:
            max_step = math.radians(shared["speed"]) * dt
            cmd = shared["cmd"]
            for i in range(NUM_ARM):  # 速度制限: 1 ステップに進める量を抑える
                cmd[i] += min(max(shared["goal"][i] - cmd[i], -max_step), max_step)
            cmd_t = torch.tensor(cmd, dtype=torch.float32, device=device)
        full_target[0, arm_ids] = cmd_t
        robot.set_joint_position_target(full_target)
        robot.write_data_to_sim()
        sim.step()
        robot.update(dt)
        camera.update(dt)

        if step % 2 == 0:
            pos = robot.data.joint_pos[0, arm_ids].cpu().numpy()
            rgb = camera.data.output["rgb"][0, ..., :3].cpu().numpy().astype(np.uint8)
            buf = io.BytesIO()
            Image.fromarray(rgb).save(buf, format="JPEG", quality=80)
            with lock:
                shared["pos"] = [float(v) for v in pos]
            shared["jpeg"] = buf.getvalue()
        if step % 600 == 0:
            print(f"[ctrl] step={step} pos[deg]={[round(math.degrees(v), 1) for v in shared['pos']]}", flush=True)
        step += 1
    server.shutdown()
    print("[ctrl] done", flush=True)


if __name__ == "__main__":
    main()
    simulation_app.close()
