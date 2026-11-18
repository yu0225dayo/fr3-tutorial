"""FR3 操作の中核(画面なし): バックエンド(シミュレーション/実機)・安全検査・禁止エリア。
画面は fr3_tk.py(コンテナ内・RViz と並べて表示)と fr3_gui.py(ブラウザ版)。

以下は fr3_gui.py(ブラウザ版)の使い方:

  python3 fr3_gui.py                 # 既定: 127.0.0.1:8096、実機への動作送信は「ドライラン」
  python3 fr3_gui.py --execute       # 実機に動作を送る(それでも、画面の安全確認と動作ごとの確認が要る)
  python3 fr3_gui.py --host 0.0.0.0  # LAN から開く場合(認証なしなので、安全な LAN だけで)

接続先の選択: 「シミュレーション」(実機に何も送らない)か「実機」。
実機の通信は、libfairino を包んだ fr3_bridge(同じフォルダ)を介する。
安全ルールは fr3-real/CLAUDE.md に従う: 既定はドライラン / 低速 / 範囲検査(切り詰めず拒否) / ログ。
"""

import json
import math
import os
import queue
import subprocess
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
LOG_DIR = os.path.join(ROOT, "logs")
LIM = json.load(open(os.path.join(ROOT, "config", "limits.json")))
J_MIN = np.array(LIM["joint_deg"]["min"]) + LIM["joint_margin_deg"]
J_MAX = np.array(LIM["joint_deg"]["max"]) - LIM["joint_margin_deg"]
IMP = LIM["impedance"]
HOME_DEG = [-10.93, -93.27, 77.22, -73.96, -90.0, -100.94]  # 初期姿勢(init_pose.csv)

os.makedirs(LOG_DIR, exist_ok=True)
log_f = open(os.path.join(LOG_DIR, time.strftime("gui_%Y%m%d_%H%M%S.jsonl")), "a")
log_lock = threading.Lock()


def log(**e):
    e["t"] = time.time()
    with log_lock:
        log_f.write(json.dumps(e, ensure_ascii=False) + "\n")
        log_f.flush()


# ---------------------------------------------------------------- シミュレーション用の運動学
CHAIN = [  # (xyz, rpy, axis) URDF fairino3_v6
    ((0, 0, 0), (0, 0, 0)), ((0, 0, 0.14), (math.pi / 2, 0, 0)), ((-0.28, 0, 0), (0, 0, 0)),
    ((-0.24001, 0, 0), (0, 0, 0)), ((0, 0, 0.102), (math.pi / 2, 0, 0)), ((0, 0, 0.102), (-math.pi / 2, 0, 0)),
]


def rot_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ np.array(
        [[1, 0, 0], [0, cr, -sr], [0, sr, cr]])


def rot_z(a):
    return np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])


def fk(q_rad):
    R, p = np.eye(3), np.zeros(3)
    for (xyz, rpy), q in zip(CHAIN, q_rad):
        p = p + R @ np.array(xyz)
        R = R @ rot_rpy(*rpy) @ rot_z(q)
    return p, R


def pose_from(p, R):
    ry = -math.asin(max(-1, min(1, R[2, 0])))
    return [*(p * 1000), math.degrees(math.atan2(R[2, 1], R[2, 2])), math.degrees(ry), math.degrees(math.atan2(R[1, 0], R[0, 0]))]


def ik(target, q0_deg):
    """減衰最小二乗の数値 IK。target=[x,y,z mm, rx,ry,rz deg]。失敗なら None"""
    pt, Rt = np.array(target[:3]) / 1000, rot_rpy(*np.radians(target[3:]))
    q = np.radians(q0_deg).astype(float)
    for _ in range(300):
        p, R = fk(q)
        dR = Rt @ R.T
        w = 0.5 * np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]])
        e = np.concatenate([pt - p, w])
        if np.linalg.norm(e[:3]) < 1e-4 and np.linalg.norm(e[3:]) < 1e-3:
            # 同じ姿勢を表す角度(±360°)のうち、出発点 q0 にいちばん近いものにする(-481° のような値を出さない)
            q0 = np.array(q0_deg, float)
            d = np.degrees(q) - q0
            return q0 + (d + 180.0) % 360.0 - 180.0
        Jm = np.zeros((6, 6))
        for i in range(6):
            dq = q.copy()
            dq[i] += 1e-6
            p2, R2 = fk(dq)
            dR2 = R2 @ R.T
            dw = 0.5 * np.array([dR2[2, 1] - dR2[1, 2], dR2[0, 2] - dR2[2, 0], dR2[1, 0] - dR2[0, 1]])
            Jm[:, i] = np.concatenate([(p2 - p) / 1e-6, dw / 1e-6])
        q += Jm.T @ np.linalg.solve(Jm @ Jm.T + 1e-4 * np.eye(6), e)
    return None


def fk_origins(q_rad):
    """各関節原点(m, 7 点: ベース〜j6)と最終回転 R。腕の点群の元"""
    R, p, pts = np.eye(3), np.zeros(3), [np.zeros(3)]
    for (xyz, rpy), q in zip(CHAIN, q_rad):
        p = p + R @ np.array(xyz)
        R = R @ rot_rpy(*rpy) @ rot_z(q)
        pts.append(p.copy())
    return pts, R


def arm_points(q_deg, tool_m=None):
    """腕の表面を代表する点(m)。関節原点の間を 5 分割し、先端(フランジから tool_m 先)も入れる"""
    tool_m = LIM.get("tool_length_mm", 100) / 1000 if tool_m is None else tool_m
    pts, R = fk_origins(np.radians(q_deg))
    pts = pts + [pts[-1] + R[:, 2] * tool_m]
    out = []
    for a, b in zip(pts[:-1], pts[1:]):
        for t in np.linspace(0, 1, 6):
            out.append(a + t * (b - a))
    return np.array(out)


ZONES_PATH = os.path.join(ROOT, "config", "zones.json")


class Zones:
    """到達してほしくない領域(ベース座標系の直方体、mm)。腕の点群が、リンク余裕 margin を足した箱に入ったら違反"""

    def __init__(self):
        self.items = []
        self.mtime = None
        self.error = None
        self.load()

    def load(self):
        """config/zones.json を読む。読めないとき(書き間違いなど)は、前の禁止エリアのままにして、error に理由を残す"""
        try:
            mtime = os.path.getmtime(ZONES_PATH)
        except OSError:
            if self.mtime is None:  # 起動したときから無い: 禁止エリアなし
                self.items, self.error = [], None
            else:  # 動いている途中で消えた(エディタの保存の途中など): 前の禁止エリアのまま
                self.error = "config/zones.json が見つからないので、前の禁止エリアのまま"
            self.mtime = None
            return
        try:
            items = json.load(open(ZONES_PATH, encoding="utf-8"))
            for z in items:  # 最低限の形を確かめる
                assert len(z["min"]) == 3 and len(z["max"]) == 3 and z["name"]
                assert all(a < b for a, b in zip(z["min"], z["max"]))
            self.items, self.error = items, None
        except (OSError, ValueError, KeyError, TypeError, AssertionError) as e:
            self.error = f"config/zones.json を読めないので、前の禁止エリアのまま: {e!r}"
        self.mtime = mtime

    def reload_if_changed(self):
        """ファイルが書き換えられていたら読み直す。読み直したら True"""
        try:
            mtime = os.path.getmtime(ZONES_PATH)
        except OSError:
            mtime = None
        if mtime == self.mtime:
            return False
        before = self.items
        self.load()
        log(action="zones_reloaded", zones=self.items, error=self.error, changed=self.items != before)
        return True

    def save(self):
        json.dump(self.items, open(ZONES_PATH, "w", encoding="utf-8"), ensure_ascii=False, indent=2)
        self.mtime = os.path.getmtime(ZONES_PATH)  # 自分で書いたものは、読み直さない
        log(action="zones_saved", zones=self.items)

    def hits(self, pts_m, margin_mm=None):
        """pts_m (N,3) が入っているエリア名の集合。pts_m の最後の点は手先(arm_points の先端)。
        エリアの "target" が "tcp" なら手先の点だけを、余裕なしで見る。既定("arm")は腕全体に余裕 margin を足して見る"""
        m = (LIM.get("link_margin_mm", 80) if margin_mm is None else margin_mm) / 1000
        out = set()
        for z in self.items:
            if not z.get("enabled", True):
                continue
            tcp_only = z.get("target", "arm") == "tcp"
            pts, mz = (pts_m[-1:], 0.0) if tcp_only else (pts_m, m)
            lo, hi = np.array(z["min"]) / 1000 - mz, np.array(z["max"]) / 1000 + mz
            if np.any(np.all((pts >= lo) & (pts <= hi), axis=1)):
                out.add(z["name"])
        return out


def pose_in_hits(zones, q_deg):
    return zones.hits(arm_points(q_deg))


# ---------------------------------------------------------------- バックエンド
class SimBackend:
    name = "sim"
    sends_to_robot = False

    def __init__(self):
        self.q = np.array(HOME_DEG, float)
        self.goal = self.q.copy()
        self.vel = 5.0
        self.imp = None
        self.connected = False
        self.lock = threading.Lock()
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        dt = 0.02
        while True:
            with self.lock:
                step = self.vel / 100 * 90.0 * dt  # 100% = 90 deg/s
                d = self.goal - self.q
                self.q += np.clip(d, -step, step)
            time.sleep(dt)

    def connect(self, ip):
        self.connected = True
        return {"ok": True, "msg": "シミュレーションに接続"}

    def disconnect(self):
        self.connected = False
        return {"ok": True}

    def state(self):
        with self.lock:
            q = self.q.copy()
            done = bool(np.max(np.abs(self.goal - self.q)) < 0.05)
        p, R = fk(np.radians(q))
        return {"joints": q.tolist(), "tcp": pose_from(p, R), "done": done, "estop": 0, "main_err": 0, "sub_err": 0, "torque": [0] * 6}

    def movej(self, j, vel):
        with self.lock:
            self.goal, self.vel = np.array(j, float), vel
        return {"ok": True}

    def movel(self, pose, vel):
        q = ik(pose, self.q)
        if q is None:
            return {"ok": False, "msg": "IK が解けない(到達できない姿勢)"}
        if np.any(q < J_MIN) or np.any(q > J_MAX):
            return {"ok": False, "msg": "IK の解が関節範囲の外"}
        return self.movej(q.tolist(), vel)  # シミュレーションでは直線でなく関節補間

    def stop(self):
        with self.lock:
            self.goal = self.q.copy()
        return {"ok": True}

    def enable(self, on):
        return {"ok": True}

    def reset(self):
        return {"ok": True}

    def impedance(self, on, ws, p):
        self.imp = {"ws": ws, **p} if on else None
        return {"ok": True, "msg": "シミュレーションでは設定を保持するだけ(外力による動きは再現しない)"}

    def drag(self, on):
        return {"ok": True, "msg": "シミュレーションでは状態を保持するだけ(手で動かす動きは再現しない)"}

    def mode(self, manual):
        return {"ok": True, "msg": f"シミュレーションでは何もしない({'手動' if manual else '自動'}モード)"}


class RealBackend:
    name = "real"
    sends_to_robot = True

    def __init__(self):
        self.proc = None
        self.connected = False
        self.q = queue.Queue()
        self.lock = threading.Lock()
        self.last_state = None

    def _call(self, line, timeout=3.0):
        with self.lock:
            if not self.proc or self.proc.poll() is not None:
                return {"ok": False, "msg": "bridge が動いていない"}
            while not self.q.empty():
                self.q.get_nowait()
            self.proc.stdin.write(line + "\n")
            self.proc.stdin.flush()
            try:
                return json.loads(self.q.get(timeout=timeout))
            except queue.Empty:
                self._kill()
                self.connected = False
                return {"ok": False, "msg": f"応答なし({timeout}s)。bridge を終了した: {line.split()[0]}"}

    def _kill(self):
        # いきなり kill せず、SDK の接続を閉じて(close)から終わらせる。応答しないときだけ kill
        if self.proc and self.proc.poll() is None:
            try:
                self.proc.stdin.write("close\nquit\n")
                self.proc.stdin.flush()
                self.proc.wait(timeout=3)
            except (OSError, subprocess.TimeoutExpired):
                self.proc.kill()
        self.proc = None

    def _reader(self, proc):
        # SDK は自分のログを標準出力にも書く。bridge の返事(1 行の JSON)だけを受け取る
        for ln in proc.stdout:
            if ln.lstrip().startswith("{"):
                self.q.put(ln)

    def connect(self, ip):
        # 事前にポートへ接続して確かめることはしない(接続してすぐ切る操作の直後に、コントローラ本体のプログラムが止まった疑いがある)。
        # 接続できるかは SDK の RPC() の結果で判断する。SDK のログは logs/sdk_*.log に残す
        self._kill()
        env = dict(os.environ, FR3_SDK_LOG=os.path.join(LOG_DIR, time.strftime("sdk_%Y%m%d_%H%M%S.log")))
        self.proc = subprocess.Popen([os.path.join(HERE, "fr3_bridge")], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     text=True, bufsize=1, env=env)
        threading.Thread(target=self._reader, args=(self.proc,), daemon=True).start()
        # RPC() の途中で bridge を強制終了しないよう、長めに待つ(実測では 3 秒ほど)
        r = self._call(f"connect {ip}", timeout=30)
        if not r.get("ok") and "msg" not in r:
            r["msg"] = f"{ip} に SDK で接続できない(err {r.get('err')})。配線・IP・PC 側の IP・コントローラの状態を確認。詳細は logs/sdk_*.log"
        self.connected = bool(r.get("ok"))
        if not self.connected:
            self._kill()
        return r

    def disconnect(self):
        if self.proc:
            self._call("close")
            self._kill()
        self.connected = False
        return {"ok": True}

    def state(self):
        r = self._call("state")
        if not r.get("ok"):
            # 関節角・手先が読めなかったとき、bridge は 0 を入れて返す。そのまま使うと、腕が伸びきった姿勢と見なされ、
            # 禁止エリアの誤検出や RViz の表示の乱れになる(非常停止の直後に起きた)ので、位置の値は捨てる
            r = {k: v for k, v in r.items() if k not in ("joints", "tcp", "flange")}
        else:
            self.last_state = r
        return r

    def movej(self, j, vel):
        return self._call("movej " + " ".join(f"{v:.4f}" for v in j) + f" {vel:.1f}")

    def movel(self, pose, vel):
        return self._call("movel " + " ".join(f"{v:.4f}" for v in pose) + f" {vel:.1f}")

    def stop(self):
        return self._call("stop")

    def enable(self, on):
        return self._call(f"enable {int(on)}")

    def reset(self):
        return self._call("reset")

    def drag(self, on):
        return self._call(f"drag {int(on)}")

    def mode(self, manual):
        """コントローラの手動/自動モードを切り替える(SDK の Mode: 0=自動、1=手動)"""
        return self._call(f"mode {1 if manual else 0}")

    def ik(self, pose, q_ref):
        """コントローラの逆運動学(今のツールの手先の位置 → 関節角 deg)。解けなければ None"""
        r = self._call("ik " + " ".join(f"{v:.4f}" for v in list(pose) + list(q_ref)))
        return np.array(r["joints"], float) if r.get("ok") and "joints" in r else None

    def fk(self, q):
        """コントローラの順運動学(関節角 deg → 今のツールの手先 [mm, deg])"""
        r = self._call("fk " + " ".join(f"{v:.4f}" for v in q))
        return np.array(r["tcp"], float) if r.get("ok") and "tcp" in r else None

    def impedance(self, on, ws, p):
        if not on:
            return self._call(f"imp 0 {ws}")
        v = lambda k: " ".join(str(x) for x in p[k])
        return self._call(f"imp 1 {ws} {v('threshold')} {v('m')} {v('b')} {v('k')} {p['max_v']} {p['max_va']} {p['max_w']} {p['max_wa']}", 5)


# ---------------------------------------------------------------- 安全検査つきの操作
class App:
    def __init__(self, execute):
        self.execute = execute
        self.sim, self.real = SimBackend(), RealBackend()
        self.be = self.sim
        self.safety_ok = False  # 画面の「非常停止・周囲の確認」チェック
        self.impedance_on = False
        self.drag_on = False  # ドラッグモード(手で動かせる状態)
        self.zones = Zones()
        self.seq = None  # インピーダンスを入れたままの移動の、動いている手順(スレッド)
        self.seq_abort = threading.Event()
        self.seq_lock = threading.Lock()  # 「■ 停止」と、手順の中の移動の送信がすれ違わないように
        self.seq_status = ""
        self.seq_deadline = None  # いまの段が終わる時刻(画面でカウントダウンする)。決まっていないときは None
        self.seq_post_s = 0.0  # 着いてから、インピーダンスを切るまでの秒数(移動中の表示に使う)

    def current_q(self):
        s = self.be.state() if self.be.connected else {}
        return np.array(s["joints"], float) if "joints" in s else None

    def path_fn(self, cmd, q0):
        """経路の途中の点を求める関数 at(t, q_seed) -> (その点を目標にしたときの命令の値, 関節角 or None) を返す。t は 0(今)〜1(目標)。
        関節移動は関節の直線補間、手先移動は手先を直線で結んで IK。失敗なら (None, 理由)"""
        kind, tgt = cmd
        if kind == "movej":
            qa, qb = np.array(q0, float), np.array(tgt, float)
            return (lambda t, seed=None: ((qa + (qb - qa) * t).tolist(), qa + (qb - qa) * t)), None
        if self.be.sends_to_robot:
            # 実機: 手先の位置はコントローラの座標(今のツール 1 番の先端)なので、順・逆運動学もコントローラで解く
            # (GUI の運動学はツールを知らず、z も約 100 mm ずれる)。関節角が出れば、禁止エリア・関節範囲の検査は GUI でできる
            a = self.be.fk(q0)
            if a is None:
                return None, "コントローラの順運動学が使えない"
            solve = self.be.ik
        else:
            p0, R0 = fk(np.radians(q0))
            a = np.array(pose_from(p0, R0))
            solve = ik
        b = np.array(tgt, float)
        b[3:] = a[3:] + (b[3:] - a[3:] + 180.0) % 360.0 - 180.0  # 向き(rx ry rz)は近い回り方で補間する(-180→83 を 263° 回さない)

        def at(t, seed):
            pose = a + (b - a) * t
            return pose.tolist(), solve(list(pose), seed)
        return at, None

    def plan_path(self, cmd, q0, n=30):
        """動作の途中の関節角の列(近似)。失敗なら (None, 理由)"""
        at, why = self.path_fn(cmd, q0)
        if at is None:
            return None, why
        q, path = np.array(q0, float), [np.array(q0, float)]
        for t in np.linspace(0, 1, n + 1)[1:]:
            _, q = at(t, q)
            if q is None:
                return None, "途中で IK が解けない(手先を直線で結べない/到達できない)"
            if np.any(q < J_MIN) or np.any(q > J_MAX):
                bad = [f"j{i + 1}={q[i]:.0f}°" for i in range(6) if not J_MIN[i] <= q[i] <= J_MAX[i]]
                return None, f"直線の途中で関節範囲を超える({', '.join(bad)})。手先の向き(rx ry rz)を変えるか、関節角度で指定して移動する"
            path.append(q.copy())
        return path, None

    def zone_clip(self, cmd, n=30):
        """禁止エリアに入る動作は、断らずに、目標をエリアの手前(余裕 zone_stop_margin_mm)に縮める。
        戻り値: (送る命令, 説明 or None)。送れないときは (None, 理由)"""
        q0 = self.current_q()
        if q0 is None:
            return None, "現在の関節角が読めない(禁止エリアの検査ができない)"
        at, why = self.path_fn(cmd, q0)
        if at is None:
            return None, why
        path, why = self.plan_path(cmd, q0, n)
        if path is None:
            return None, why
        hits = [self.zones.hits(arm_points(q)) for q in path]
        bad = [i for i, h in enumerate(hits) if h]
        if not bad:
            return cmd, None
        names = ", ".join(sorted(set().union(*hits)))
        if bad[0] == 0:
            if bad == list(range(len(bad))) and not hits[-1]:  # いま中にいて、出ていく動きだけは許す
                return cmd, None
            return None, f"いま禁止エリアの中にいる({names})。エリアの外へ出る動きだけ受け付ける"
        # 入る直前の点(i-1)と入った点(i)のあいだを二分探索して、境目を探す
        i = bad[0]
        ts = np.linspace(0, 1, n + 1)
        lo, hi, q_lo = ts[i - 1], ts[i], path[i - 1]
        for _ in range(12):
            mid = (lo + hi) / 2
            _, qm = at(mid, q_lo)
            if qm is not None and not self.zones.hits(arm_points(qm)):
                lo, q_lo = mid, qm
            else:
                hi = mid
        # 境目から、手先の進んだ距離で zone_stop_margin_mm だけ手前に戻す
        tip = lambda q: arm_points(q)[-1] * 1000
        speed = np.linalg.norm(tip(path[i]) - tip(path[i - 1])) / (ts[i] - ts[i - 1])  # mm / t
        t_stop = lo - LIM.get("zone_stop_margin_mm", 20) / max(speed, 1e-6)
        if t_stop <= 0.01:
            return None, f"すでに禁止エリア({names})の手前ぎりぎりにいるので、この向きには動かない"
        tgt, q_stop = at(t_stop, path[max(i - 1, 0)] if t_stop < ts[i - 1] else q_lo)
        if q_stop is None:
            return None, "禁止エリアの手前の点が求められない"
        moved = np.linalg.norm(tip(q_stop) - tip(path[0]))
        full = np.linalg.norm(tip(path[-1]) - tip(path[0]))
        note = f"このままだと禁止エリア({names})に入るので、手前で止める(手先 {moved:.0f} mm 動いて止まる。元の目標までは {full:.0f} mm)"
        return (cmd[0], [round(float(v), 4) for v in tgt]), note

    def guard(self):
        """腕が禁止エリアの中にいるか(監視用)。名前の集合"""
        q = self.current_q()
        return set() if q is None else self.zones.hits(arm_points(q))

    def dry(self):
        return self.be.sends_to_robot and not self.execute

    def gate(self, act, payload):
        """動作を送ってよいかを決める。(None, 理由) なら拒否。('dry', ..) ならドライラン"""
        if not self.be.connected:
            return "not_connected", "未接続"
        if self.be.sends_to_robot and not self.safety_ok:
            return "refuse", "画面の「非常停止の場所と周囲の確認」にチェックが要る"
        if self.dry():
            return "dry", "ドライラン(サーバを --execute で起動していないので、実機には送らない)"
        return "ok", ""

    def plan_move(self, d, allow_imp=False):
        """移動の命令を検査して、送る命令を決める(禁止エリアに入るなら手前に縮める)。送らない。
        戻り値: {"ok": True, "cmd": (種類, 値), "vel": 速度, "note": 説明 or None} か {"ok": False, "msg": 理由}"""
        if not self.be.connected:
            return {"ok": False, "msg": "未接続です。先に「接続」を押してください"}
        kind = d["kind"]
        vel = float(d.get("vel", LIM["default_vel_percent"]))
        if not 0 < vel <= LIM["max_vel_percent"]:
            return {"ok": False, "msg": f"速度は 0〜{LIM['max_vel_percent']}% (上限は config/limits.json)"}
        if self.impedance_on and not allow_imp:  # allow_imp: インピーダンスを入れたままの手順の中で使うときだけ
            return {"ok": False, "msg": "インピーダンス制御中は移動しない(先に停止する)"}
        if self.drag_on:
            return {"ok": False, "msg": "ドラッグモード中は移動しない(先にドラッグモードを抜ける)"}
        if kind == "joint":
            j = np.array([float(x) for x in d["joints"]])
            if len(j) != 6 or np.any(j < J_MIN) or np.any(j > J_MAX):
                bad = [f"j{i + 1}" for i in range(6) if not J_MIN[i] <= j[i] <= J_MAX[i]]
                return {"ok": False, "msg": f"関節範囲の外: {bad}(範囲 {J_MIN.round(0).tolist()}〜{J_MAX.round(0).tolist()})"}
            cmd = ("movej", j.tolist())
        else:
            p = [float(x) for x in d["pose"]]
            t = LIM["tcp_mm"]
            if not (t["x"][0] <= p[0] <= t["x"][1] and t["y"][0] <= p[1] <= t["y"][1] and t["z"][0] <= p[2] <= t["z"][1]) \
                    or math.hypot(*p[:3]) > t["max_radius"]:
                return {"ok": False, "msg": f"手先位置が作業範囲の外 (config/limits.json の tcp_mm)"}
            cmd = ("movel", p)
        cmd2, note = self.zone_clip(cmd)
        if cmd2 is None:
            log(action="move_refused_zone", cmd=cmd[0], target=cmd[1], why=note)
            return {"ok": False, "msg": note}
        if note:
            log(action="move_clipped_zone", cmd=cmd[0], target=cmd[1], clipped=cmd2[1], note=note)
        return {"ok": True, "cmd": cmd2, "vel": vel, "note": note}

    def move(self, d):
        pm = self.plan_move(d)
        if not pm.get("ok"):
            return pm
        cmd, vel, note = pm["cmd"], pm["vel"], pm["note"]
        verdict, why = self.gate("move", d)
        log(action="move", backend=self.be.name, cmd=cmd[0], target=cmd[1], vel=vel, verdict=verdict, why=why)
        if verdict == "dry":
            return {"ok": True, "dry_run": True, "msg": f"{why}: {cmd[0]} {np.round(cmd[1], 2).tolist()} 速度{vel}%" + (f"\n{note}" if note else "")}
        if verdict != "ok":
            return {"ok": False, "msg": why}
        r = getattr(self.be, cmd[0])(cmd[1], vel)
        log(action="move_result", result=r)
        if note and r.get("ok"):
            r = {**r, "msg": note}
        return r

    def drag(self, on):
        """ドラッグモード(手で腕を動かせる状態)に入る/抜ける。抜けるのはいつでも通す"""
        if on and self.impedance_on:
            return {"ok": False, "msg": "インピーダンス制御中はドラッグモードに入らない(先に停止する)"}
        verdict, why = self.gate("drag", {})
        if not on and verdict == "refuse":
            verdict = "ok"
        log(action="drag", backend=self.be.name, on=on, verdict=verdict, why=why)
        if verdict == "dry":
            return {"ok": True, "dry_run": True, "msg": f"{why}: ドラッグモード {'開始' if on else '終了'}"}
        if verdict != "ok":
            return {"ok": False, "msg": why}
        r = self.be.drag(on)
        if r.get("ok"):
            self.drag_on = on
        log(action="drag_result", result=r)
        return r

    def plan_move_imp(self, d):
        """インピーダンスを入れたままの移動の、送る命令を決める(送らない)。
        インピーダンス中は、手先の直線移動(MoveL)は受け付けられる(戻り値 0)が、腕が動かない(2026-11-18 実機で確認)。
        関節移動(MoveJ)は動くので、手先の目標は IK で関節角にして、関節移動として検査し直す(経路は直線ではなくなる)"""
        pm = self.plan_move(d)
        if not pm.get("ok") or pm["cmd"][0] != "movel":
            return pm
        q0 = self.current_q()
        if q0 is None:
            return {"ok": False, "msg": "今の関節角が読めないので、関節移動に直せません"}
        tgt = pm["cmd"][1]
        q = self.be.ik(tgt, q0) if hasattr(self.be, "ik") else ik(tgt, q0)
        if q is None:
            return {"ok": False, "msg": "目標の手先の IK が解けないので、関節移動に直せません"}
        pm = self.plan_move({"kind": "joint", "vel": pm["vel"], "joints": np.asarray(q, float).tolist()})
        if not pm.get("ok"):
            return pm
        conv = "インピーダンス中は手先の直線移動が動かないため、関節移動にします(経路は直線ではありません)"
        return {**pm, "note": " / ".join(x for x in (conv, pm["note"]) if x)}

    def move_with_imp(self, d, imp_d, pre_s=1.0, post_s=5.0, timeout_s=180.0):
        """インピーダンス制御を入れたまま移動する(コントローラのインピーダンスは、軌道があるときだけ効くため)。
        手順: インピーダンス開始 → pre_s 秒待つ → 目標へ移動 → 着いてから post_s 秒待つ → インピーダンス終了。
        移動の検査(速度・範囲・禁止エリア)と、インピーダンスの値の検査は、それぞれ単独のときと同じ。
        手順は別のスレッドで進む(画面は止まらず、「■ 停止」で abort_seq() すれば途中でやめる)"""
        if self.seq and self.seq.is_alive():
            return {"ok": False, "msg": "インピーダンスを入れたままの移動を、実行中です(終わるか「■ 停止」を押してから)"}
        if self.impedance_on:
            return {"ok": False, "msg": "インピーダンス制御がすでに入っています(先に停止する)"}
        pm = self.plan_move_imp(d)
        if not pm.get("ok"):
            return pm
        ic = self.imp({**imp_d, "on": True, "check_only": True})
        if not ic.get("ok"):
            return ic
        cmd, vel, note = pm["cmd"], pm["vel"], pm["note"]
        verdict, why = self.gate("move", d)
        log(action="move_with_imp", backend=self.be.name, cmd=cmd[0], target=cmd[1], vel=vel, ws=int(imp_d["ws"]),
            params=ic["params"], pre_s=pre_s, post_s=post_s, verdict=verdict, why=why)
        if verdict == "dry":
            return {"ok": True, "dry_run": True, "msg": f"{why}: インピーダンス開始 → {pre_s:g} 秒後に {cmd[0]} {np.round(cmd[1], 2).tolist()} "
                    f"速度{vel}% → 着いて {post_s:g} 秒後にインピーダンス終了" + (f"\n{note}" if note else "")}
        if verdict != "ok":
            return {"ok": False, "msg": why}
        self.seq_abort.clear()
        self.seq_status, self.seq_deadline = "インピーダンス開始中", None
        self.seq = threading.Thread(target=self._seq_run, args=(cmd, vel, imp_d, pre_s, post_s, timeout_s), daemon=True)
        self.seq.start()
        return {"ok": True, "msg": f"インピーダンスを入れて、{pre_s:g} 秒後に移動します(着いて {post_s:g} 秒後に終了)" + (f"\n{note}" if note else "")}

    def _seq_run(self, cmd, vel, imp_d, pre_s, post_s, timeout_s, legs=None, hold_s=0.0, skip_first=False):
        """インピーダンス開始 → pre_s 秒 → cmd へ移動 → (legs があれば hold_s 秒のあいだ legs を順にくり返し、最後に cmd へ戻る)
        → post_s 秒 → インピーダンス終了"""
        ws = int(imp_d["ws"])
        self.seq_post_s = post_s

        def finish(status):
            if self.impedance_on:
                r = self.imp({"on": False, "ws": ws})
                log(action="move_with_imp_imp_off", result=r)
                if not r.get("ok"):
                    status += f" / インピーダンスを切れませんでした(err {r.get('err')})。非常停止を押してください"
            self.seq_status = status
            self.seq_deadline = None
            log(action="move_with_imp_end", status=status)
            return False

        def go(c, label, settle_n=3):
            """1 回の移動を送り、着くまで待つ。やめたときは False"""
            q_start = self.current_q()
            with self.seq_lock:  # 「■ 停止」が押されたあとには、移動を送らない
                if self.seq_abort.is_set():
                    return finish(f"停止されました({label}の前)")
                r = getattr(self.be, c[0])(c[1], vel)
            log(action="move_with_imp_move", cmd=c[0], target=c[1], vel=vel, label=label, result=r)
            if not r.get("ok"):
                return finish(f"移動の命令が断られました({label}、err {r.get('err')}、{r.get('msg', '')})")
            t0, done_n = time.time(), 0
            if self.seq_abort.wait(0.3):  # 送った直後は、まだ「止まっている」と読めることがあるので、少し待ってから見る
                return finish(f"停止されました({label}中)")
            while True:
                st = self.be.state()
                done_n = done_n + 1 if st.get("done", False) else 0
                if done_n >= settle_n:
                    # 命令は受け付けられたのに、腕がまったく動かなかった(MoveL をインピーダンス中に送ったときに起きた)
                    q_end = self.current_q()
                    if c[0] == "movej" and q_start is not None and q_end is not None:
                        far = float(np.max(np.abs(np.asarray(c[1]) - q_start)))
                        moved = float(np.max(np.abs(q_end - q_start)))
                        if far > 1.0 and moved < 0.2:
                            log(action="move_with_imp_not_moved", label=label, far_deg=far, moved_deg=moved)
                            return finish(f"{label}: 命令は受け付けられたが、腕が動きませんでした(目標まで {far:.1f}°)")
                    return True
                if time.time() - t0 > timeout_s:
                    self.be.stop()
                    return finish(f"{label}が {timeout_s:g} 秒たっても終わらないので、止めました")
                if self.seq_abort.wait(0.1):
                    return finish(f"停止されました({label}中)")

        r = self.imp({**imp_d, "on": True, "confirm": True})
        if not r.get("ok"):
            self.seq_status = f"インピーダンスを開始できませんでした(err {r.get('err')}) / 移動しません"
            log(action="move_with_imp_end", status=self.seq_status)
            return
        self.seq_status, self.seq_deadline = "インピーダンス ON・移動を始めるまで", time.time() + pre_s
        if self.seq_abort.wait(pre_s):
            return finish("停止されました(移動の前)")
        # いまいる点への移動(長さ 0)は、インピーダンス中は速度リミット超過のエラーになるので送らない
        if not skip_first:
            self.seq_status, self.seq_deadline = f"インピーダンス ON・目標へ移動中(着いてから {post_s:g} 秒で終了)", None
            if not go(cmd, "目標への移動"):
                return
        if legs:
            t_end, n = time.time() + hold_s, 0
            self.seq_deadline = t_end
            while time.time() < t_end:
                for c in legs:
                    n += 1
                    self.seq_status = f"インピーダンス ON・保持中(行き来 {n} 回)。保持が終わるまで"
                    if not go(c, "保持の行き来", settle_n=1):  # 行き来の合間に止まる時間を短くする
                        return
            self.seq_status, self.seq_deadline = f"インピーダンス ON・目標へ戻り中(着いてから {post_s:g} 秒で終了)", None
            if not go(cmd, "目標へ戻る"):
                return
        # 着いたあとの待ち: 外力でずれたら、インピーダンスを入れたまま目標へ送り直す(k=0 では自分では戻らないため。
        # 送り直しは長さ 0 にならない(ずれている)ので、速度リミット超過のエラーにはならない)
        tol = float(LIM.get("imp_return_tol_deg", 0.3))
        t_end, n_ret = time.time() + post_s, 0
        self.seq_deadline = t_end
        # ずれの戻しは、目標そのものへは送らない。インピーダンスのずれは、コントローラの指令(目標)の上に乗っているので、
        # 目標へ送ると指令の上では長さ 0 の移動になり、故障 1-26 になった(2026-11-18)。そこで、目標の ±amp の点へ
        # 交互に送り(「今の位置を保持」と同じ。この行き来で、ずれが戻ることを確認済み)、ずれが小さくなったらやめる
        amp = float(LIM.get("imp_return_amp_deg", 0.3))
        ret_legs = []
        if cmd[0] == "movej":
            for sg in (1, -1):
                pm = self.plan_move({"kind": "joint", "vel": vel, "joints": (np.asarray(cmd[1]) + sg * amp).tolist()}, allow_imp=True)
                if pm.get("ok") and not pm["note"]:
                    ret_legs.append(pm["cmd"])
            if len(ret_legs) < 2:
                ret_legs = []
                log(action="move_with_imp_no_return_legs", why="行き来の点が検査を通らないので、ずれの戻しはしない")
        while True:
            left = t_end - time.time()
            if left <= 0:
                break
            self.seq_status = f"着きました。インピーダンス ON(ずれの戻し {n_ret} 回)。インピーダンス終了まで"
            q = self.current_q()
            # 行き来の点そのものが目標から amp ずれているので、amp + tol を超えたときだけ戻す(残りは最後の仕上げで合わせる)
            if ret_legs and q is not None and float(np.max(np.abs(np.asarray(cmd[1]) - q))) > amp + tol:
                n_ret += 1
                if not go(ret_legs[n_ret % 2], "ずれの戻し", settle_n=1):
                    return
                continue
            if self.seq_abort.wait(min(0.3, max(left, 0.01))):
                return finish("停止されました(着いたあとの待ち)")
        self.seq_deadline = None
        if self.impedance_on:
            r = self.imp({"on": False, "ws": ws})
            log(action="move_with_imp_imp_off", result=r)
            if not r.get("ok"):
                return finish(f"インピーダンスを切れませんでした(err {r.get('err')})。非常停止を押してください")
        if self.seq_abort.wait(0.5):  # インピーダンスを切ったあと、少し落ち着かせる
            return finish("停止されました(仕上げの前)")
        # 最後の仕上げ: インピーダンスを切ったあと、まだずれていれば(行き来の点にいるときも)、ふつうの移動(検査つき)で目標へ合わせる
        q = self.current_q()
        if cmd[0] == "movej" and q is not None and float(np.max(np.abs(np.asarray(cmd[1]) - q))) > tol:
            pm = self.plan_move({"kind": "joint", "vel": vel, "joints": list(cmd[1])})
            if not pm.get("ok") or pm["note"]:
                return finish(f"インピーダンス OFF。目標からずれていますが、仕上げの移動は検査で断りました: {pm.get('msg') or pm.get('note')}")
            self.seq_status, self.seq_deadline = "インピーダンス OFF・目標へ仕上げの移動中", None
            if not go(pm["cmd"], "仕上げの移動"):
                return
        q = self.current_q()
        err = float(np.max(np.abs(np.asarray(cmd[1]) - q))) if (cmd[0] == "movej" and q is not None) else 0.0
        finish(f"終わりました(インピーダンス OFF、目標とのずれ 最大 {err:.2f}°、ずれの戻し {n_ret} 回)")

    def hold_with_imp(self, target_joints, imp_d, amp_deg=0.3, hold_s=30.0, vel=2.0, pre_s=1.0, post_s=10.0, timeout_s=60.0):
        """実質の静止状態でインピーダンスを効かせる(コントローラのインピーダンスは、軌道があるときだけ効くため)。
        手順: インピーダンス開始 → 目標の関節角へ移動 → 全関節を ±amp_deg だけ小さく行き来させ続ける(hold_s 秒)
        → 目標へ戻る → インピーダンスを入れたまま post_s 秒(既定 10 秒)待つ → インピーダンス終了。行き来の点も、ふつうの移動と同じ検査(範囲・禁止エリア)を通す"""
        if self.seq and self.seq.is_alive():
            return {"ok": False, "msg": "インピーダンスを入れたままの手順を、実行中です(終わるか「■ 停止」を押してから)"}
        if self.impedance_on:
            return {"ok": False, "msg": "インピーダンス制御がすでに入っています(先に停止する)"}
        amp_max = float(LIM.get("hold_amp_max_deg", 1.0))
        amp_min = float(LIM.get("hold_amp_min_deg", 0.3))
        if not amp_min <= amp_deg <= amp_max:
            return {"ok": False, "msg": f"行き来の幅は {amp_min:g}〜{amp_max:g}°(config/limits.json の hold_amp_min_deg / hold_amp_max_deg)。"
                    "小さすぎると、押したときに故障になる(0.001° で故障 1-23 になった)"}
        if not 0 < hold_s <= 300:
            return {"ok": False, "msg": "保持の時間は 0〜300 秒"}
        q = np.array(target_joints, float)
        cmds = []
        for qq in (q, q + amp_deg, q - amp_deg):
            pm = self.plan_move({"kind": "joint", "vel": vel, "joints": qq.tolist()})
            if not pm.get("ok"):
                return pm
            if pm["note"]:  # 禁止エリアで縮められた点では、行き来が狂うので断る
                return {"ok": False, "msg": f"目標か行き来の点が禁止エリアにかかるので、やめます: {pm['note']}"}
            cmds.append(pm["cmd"])
        ic = self.imp({**imp_d, "on": True, "check_only": True})
        if not ic.get("ok"):
            return ic
        q_now = self.current_q()
        skip_first = q_now is not None and float(np.max(np.abs(q_now - q))) < 0.05
        verdict, why = self.gate("move", {})
        log(action="hold_with_imp", backend=self.be.name, target=q.tolist(), amp_deg=amp_deg, hold_s=hold_s, vel=vel,
            ws=int(imp_d["ws"]), params=ic["params"], verdict=verdict, why=why)
        if verdict == "dry":
            return {"ok": True, "dry_run": True, "msg": f"{why}: インピーダンス開始 → 関節 {np.round(q, 2).tolist()} へ → "
                    f"±{amp_deg:g}° を {hold_s:g} 秒行き来 → 戻ってインピーダンス終了"}
        if verdict != "ok":
            return {"ok": False, "msg": why}
        self.seq_abort.clear()
        self.seq_status, self.seq_deadline = "インピーダンス開始中", None
        self.seq = threading.Thread(target=self._seq_run, args=(cmds[0], vel, imp_d, pre_s, post_s, timeout_s),
                                    kwargs={"legs": cmds[1:], "hold_s": hold_s, "skip_first": skip_first}, daemon=True)
        self.seq.start()
        return {"ok": True, "msg": f"インピーダンスを入れて、±{amp_deg:g}° の小さな行き来で {hold_s:g} 秒保持します(戻ってから {post_s:g} 秒待って終了)"}

    def safe_idle(self):
        """腕が止まり、インピーダンス・ドラッグ・手順がすべて終わっていて、エラーもない(=次の移動を始めてよい)とき (True, "")。
        そうでなければ (False, 理由)"""
        if not self.be.connected:
            return False, "未接続"
        if self.seq and self.seq.is_alive():
            return False, "インピーダンスを入れたままの手順を実行中"
        if self.impedance_on:
            return False, "インピーダンス制御中"
        if self.drag_on:
            return False, "ドラッグモード中"
        st = self.be.state()
        if "joints" not in st:
            return False, "状態が読めない"
        if not st.get("done", True):
            return False, "動作中"
        if st.get("main_err"):
            return False, f"コントローラのエラー {st.get('main_err')}-{st.get('sub_err')}"
        if st.get("estop"):
            return False, "非常停止中"
        return True, ""

    def reset_error(self):
        """コントローラの故障をリセットし(ResetAllError)、インピーダンスも切り直す。腕は動かさない"""
        if not self.be.connected:
            return {"ok": False, "msg": "未接続"}
        if self.seq and self.seq.is_alive():
            return {"ok": False, "msg": "インピーダンスを入れたままの手順を、実行中です(先に「■ 停止」)"}
        r = self.be.reset()
        log(action="reset_error", result=r)
        if not r.get("ok"):
            return {**r, "msg": f"リセットできませんでした(err {r.get('err')})。Web 画面でエラーをクリアしてください"}
        if self.impedance_on:
            ri = self.imp({"on": False, "ws": 0})
            log(action="reset_error_imp_off", result=ri)
            if not ri.get("ok"):
                return {**ri, "msg": f"リセットはできたが、インピーダンスを切れませんでした(err {ri.get('err')})"}
        return {"ok": True, "msg": "コントローラのエラーをリセットしました" + ("(インピーダンスも切りました)" if not self.impedance_on else "")}

    def abort_seq(self):
        """インピーダンスを入れたままの移動を、途中でやめさせる(このあと呼ぶ側で、インピーダンスを切り、動作を止める)"""
        with self.seq_lock:
            self.seq_abort.set()

    def set_mode(self, manual):
        """手動/自動モードの切り替え。腕は動かさないが、コントローラの状態を変えるので、動作と同じ検査を通す。
        インピーダンス制御中・ドラッグ中・動作中は切り替えない"""
        if self.seq and self.seq.is_alive():
            return {"ok": False, "msg": "インピーダンスを入れたままの移動を、実行中です"}
        if self.impedance_on:
            return {"ok": False, "msg": "インピーダンス制御中はモードを切り替えない(先に停止する)"}
        if self.drag_on:
            return {"ok": False, "msg": "ドラッグモード中はモードを切り替えない(先にドラッグモードを抜ける)"}
        st = getattr(self.be, "last_state", None) or {}
        if self.be.connected and st and not st.get("done", True):
            return {"ok": False, "msg": "動作中はモードを切り替えない(止まってから)"}
        verdict, why = self.gate("mode", {})
        name = "手動" if manual else "自動"
        log(action="mode", backend=self.be.name, manual=manual, verdict=verdict, why=why)
        if verdict == "dry":
            return {"ok": True, "dry_run": True, "msg": f"{why}: {name}モードへ切り替え"}
        if verdict != "ok":
            return {"ok": False, "msg": why}
        r = self.be.mode(manual)
        log(action="mode_result", result=r)
        if r.get("ok") and not r.get("msg"):
            r = {**r, "msg": f"{name}モードへ切り替えました"}
        return r

    def imp(self, d):
        on, ws = bool(d["on"]), int(d["ws"])
        p = None
        if on and self.drag_on:
            return {"ok": False, "msg": "ドラッグモード中はインピーダンス制御を入れない(先にドラッグモードを抜ける)"}
        if on:
            if self.dry() is False and self.be.sends_to_robot and not d.get("confirm") and not d.get("check_only"):
                return {"ok": False, "msg": "確認が必要"}
            th = [float(x) for x in d["threshold"]]
            m, b, k = ([float(x) for x in d[n]] for n in ("m", "b", "k"))
            if ws == 0:
                lo, hi = IMP["joint_threshold_nm"]["min"], IMP["joint_threshold_nm"]["max"]
                mv, mva = IMP["joint_max_v_deg_s"], IMP["joint_max_va_deg_s2"]
                mw, mwa = mv, mva
            else:
                lo, hi = IMP["cart_threshold"]["min"], IMP["cart_threshold"]["max"]
                mv, mva, mw, mwa = IMP["max_v_mm_s"], IMP["max_va_mm_s2"], IMP["max_w_deg_s"], IMP["max_wa_deg_s2"]
            mx_v, mx_va, mx_w, mx_wa = (float(d[n]) for n in ("max_v", "max_va", "max_w", "max_wa"))
            errs = []
            if any(not lo[i] <= th[i] <= hi[i] for i in range(6)):
                errs.append(f"力の閾値はマニュアルの範囲 {lo}〜{hi}")
            if any(not 0 < m[i] <= IMP["m_max"][i] for i in range(6)) or any(not 0 <= b[i] <= IMP["b_max"][i] for i in range(6)):
                errs.append("m / b が上限(config)を超える")
            if any(v != 0 for v in k):
                errs.append("k は 0 のみ許可(config の k_max)")
            if not (0 < mx_v <= mv and 0 < mx_va <= mva and 0 < mx_w <= mw and 0 < mx_wa <= mwa):
                errs.append(f"最大速度・加速度が上限を超える (v≤{mv}, va≤{mva}, w≤{mw}, wa≤{mwa})")
            if errs:
                return {"ok": False, "msg": " / ".join(errs)}
            p = {"threshold": th, "m": m, "b": b, "k": k, "max_v": mx_v, "max_va": mx_va, "max_w": mx_w, "max_wa": mx_wa}
            if d.get("check_only"):  # 値の検査だけ(送らない)。インピーダンスを入れたままの移動の前に使う
                return {"ok": True, "params": p}
        verdict, why = self.gate("imp", d)
        if not on and verdict in ("refuse",):  # 停止はいつでも通す
            verdict = "ok"
        log(action="impedance", backend=self.be.name, on=on, ws=ws, params=p, verdict=verdict, why=why)
        if verdict == "dry":
            return {"ok": True, "dry_run": True, "msg": f"{why}: インピーダンス {'開始' if on else '停止'}"}
        if verdict != "ok":
            return {"ok": False, "msg": why}
        r = self.be.impedance(on, ws, p)
        if r.get("ok"):
            self.impedance_on = on
        log(action="impedance_result", result=r)
        return r


