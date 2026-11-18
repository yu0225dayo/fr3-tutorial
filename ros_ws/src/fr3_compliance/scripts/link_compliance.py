#!/usr/bin/env python3
"""リンクの途中に力をかけて、関節がばねのように動く（関節インピーダンス）デモ。
/link_force に [区間, FX, FY, FZ, 位置, モード] [N] を流すと、その区間の指定位置に力がかかる。
  モード 0（省略時）: 力は base_link 座標系 [x, y, z]
  モード 1: 力は腕の向きに合わせた [真横, 前後, 上]。真横 = 腕が動く平面（j2 の軸）に垂直な水平方向。腕が回っても、真横のまま
  区間 s = 関節 j_s と j_(s+1) の間のリンク（例: 3 = j3〜j4 の間 = 前腕）。6 = 手首の先（j6 の原点）
  位置 = 区間の中の位置。0 = j_s 側（根元）、1 = j_(s+1) 側（先端）。省略すると 0.5（真ん中）
力 F を関節トルク τ = Jᵀ F に変換し、各関節が  m q̈ + d q̇ + k (q - q_ref) = τ  で動く。
その点より先の関節（j_(s+1) 以降）は、その点を動かさないのでトルクを受けず、動かない。
計算した関節角を /forward_position_controller/commands に出す。

ランダムデモ（パラメータ random_demo を true にする。docker/random_demo.sh on）:
  初期姿勢 → ランダムな姿勢へ動く → 押す位置・向き・大きさもランダムに選んで数秒押す → 初期姿勢へ戻る、を繰り返す。
  しきい値を超える押し方になるまで選び直す（全部「動かない押し方」にならないように）。試行は logs/random_trials.jsonl に残す。

モデル（パラメータ model）:
  fairino : 実機のインピーダンス制御（マニュアル 9.33 節、関節空間）の再現。
            関節トルク |τ| がしきい値を超えた関節だけが、力の向きによけて動く（m ẍ + b ẋ + k x = τ - しきい値、最大速度・加速度で制限）。
            しきい値を下回ると元の姿勢に戻る（戻りの速さは仮の値）。m・b・k の単位や内部計算は不明なので、動きの速さは実機と一致しない。
  spring  : しきい値なしの単純なばね・ダンパー（最初に作ったもの）。"""
import json
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float64MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray

JOINTS = [f"j{i}" for i in range(1, 7)]
Q_REF = [-0.19073000463318587, -1.6277927678125543, 1.3478830856525392, -1.2907574813220124, -1.570728016712936, -1.7617578932848648]
MASS, STIFF, ZETA = 1.0, 20.0, 3.0   # 慣性[kg m²]、ばね[Nm/rad]、減衰比（3 で約 1.3 秒の時定数でゆっくり戻る）
DAMP = 2 * ZETA * (MASS * STIFF) ** 0.5
FORCE_SCALE = 0.008  # 矢印: m / N（50N で 40cm）
TIMEOUT = 0.5
DT = 0.01

# 実機（マニュアル 9.33 関節空間）の推奨値。j1〜j3 と j4〜j6 で値が分かれる
FAIRINO_THR = np.array([10.0] * 3 + [1.0] * 3)    # 力のしきい値 [Nm]（範囲 j1-3: 10〜50、j4-6: 1〜10。デモでは最小値）
FAIRINO_M = np.array([0.04] * 3 + [0.01] * 3)
FAIRINO_B = np.array([0.1] * 3 + [0.08] * 3)
FAIRINO_K = np.zeros(6)
FAIRINO_VMAX = np.radians(50.0)    # 最大速度 50°/s
FAIRINO_AMAX = np.radians(50.0)    # 最大加速度 50°/s²
T_RETURN = 2.0                     # 戻りの時定数 [s]（既定値。大きいほどゆっくり戻る。実機の戻り方は不明）


def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                     [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                     [-sp, cp*sr, cp*cr]])


def axis_rot(a, q):
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * K @ K


class LinkCompliance(Node):
    def __init__(self):
        super().__init__("link_compliance")
        self.model = self.declare_parameter("model", "fairino").value
        self.declare_parameter("threshold_scale", 1.0)  # しきい値の倍率。1.0=実機の最小値（10Nm / 1Nm）。小さくすると、小さい力で動く（デモ用）
        self.declare_parameter("random_demo", False)
        self.declare_parameter("delay_before_push", 2.0)   # 初期姿勢に着いてから、力をかけ始めるまで止まる時間 [s]
        self.mode, self.pair_count = 0, 0   # mode 0: 力は base_link 座標系 / 1: 力は [真横, 前後, 上] で、腕の向きに合わせる
        self.pair = None   # 小さい力（動かない）のあとに、同じ位置・向きで大きい力をかけるための記憶
        self.declare_parameter("random_pose", False)  # true にすると、試行ごとに姿勢もランダムに選ぶ（既定は初期姿勢のまま、力だけランダム）
        self.rng = np.random.default_rng()
        self.t_ok = None
        self.rstate, self.rt0, self.trial, self.push_len, self.max_disp, self.cur = "idle", 0.0, 0, 3.0, 0.0, {}
        self.create_timer(0.05, self.rand_step)
        self.t_return = self.declare_parameter("return_time", T_RETURN).value  # 動かしながら変えられる: ros2 param set
        self.chain = None  # [(xyz, rpy, axis)]
        self.tau = np.zeros(6)
        self.over = np.zeros(6, dtype=bool)
        self.q_ref = np.array(Q_REF)
        self.q = self.q_ref.copy()
        self.qd = np.zeros(6)
        self.seg, self.frac, self.force, self.last = 3, 0.5, np.zeros(3), self.get_clock().now()
        self.shown = False
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/robot_description", self.on_urdf, latched)
        self.create_subscription(Float64MultiArray, "/link_force", self.on_force, 10)
        self.cmd = self.create_publisher(Float64MultiArray, "/forward_position_controller/commands", 10)
        self.mk = self.create_publisher(MarkerArray, "force_markers", 10)
        self.status = self.create_publisher(Marker, "status_marker", 10)
        self.create_timer(DT, self.step)
        self.create_timer(0.1, self.draw)

    def on_urdf(self, msg):
        root = ET.fromstring(msg.data)
        chain = []
        for name in JOINTS:
            j = next(e for e in root.iter("joint") if e.get("name") == name)
            o = j.find("origin")
            xyz = [float(x) for x in (o.get("xyz", "0 0 0")).split()]
            r = [float(x) for x in (o.get("rpy", "0 0 0")).split()]
            ax = [float(x) for x in j.find("axis").get("xyz").split()]
            chain.append((np.array(xyz), r, np.array(ax)))
        self.chain = chain
        lim = []
        for name in JOINTS:
            j = next(e for e in root.iter("joint") if e.get("name") == name).find("limit")
            lim.append((float(j.get("lower")), float(j.get("upper"))) if j is not None and j.get("lower") else (-np.inf, np.inf))
        self.lim = np.array(lim)

    def kinematics(self, q):
        """各関節の原点 o_i と回転軸 a_i（どちらも base_link 座標系）"""
        R, p, os, As = np.eye(3), np.zeros(3), [], []
        for (xyz, r, ax), qi in zip(self.chain, q):
            p = p + R @ xyz
            R = R @ rpy(*r)
            os.append(p.copy())
            As.append(R @ (ax / np.linalg.norm(ax)))
            R = R @ axis_rot(ax, qi)
        return os, As

    def point(self, q, seg, frac):
        os, As = self.kinematics(q)
        if seg >= 6:
            return os[5], os, As
        return os[seg - 1] + frac * (os[seg] - os[seg - 1]), os, As

    def on_force(self, msg):
        d = list(msg.data) + [0.0] * 4
        self.seg = int(min(max(d[0], 1), 6))
        self.force = np.clip(np.array(d[1:4]), -50, 50)
        self.frac = float(np.clip(msg.data[4], 0.0, 1.0)) if len(msg.data) >= 5 else 0.5
        self.mode = int(msg.data[5]) if len(msg.data) >= 6 else 0
        self.last = self.get_clock().now()

    def to_world(self, v, mode, q):
        """モード 1 の [真横, 前後, 上] を、姿勢 q のときの base_link 座標系の力に直す"""
        v = np.asarray(v, float)
        if mode == 0:
            return v
        _, As = self.kinematics(q)
        side = np.array([As[1][0], As[1][1], 0.0])   # j2 の軸（腕が動く平面の法線）の水平成分
        side /= np.linalg.norm(side)
        up = np.array([0.0, 0.0, 1.0])
        return v[0] * side + v[1] * np.cross(side, up) + v[2] * up

    def world_force(self):
        return self.to_world(self.force, self.mode, self.q)

    def active(self):
        return (self.get_clock().now() - self.last).nanoseconds * 1e-9 < TIMEOUT and np.linalg.norm(self.force) > 1e-6

    def step(self):
        if self.chain is None:
            return
        tau = np.zeros(6)
        if self.active():
            p, os, As = self.point(self.q, self.seg, self.frac)
            F = self.world_force()
            for i in range(self.seg):  # その点より手前の関節 j1..j_seg だけが、点を動かす
                tau[i] = np.cross(As[i], p - os[i]) @ F
        self.tau = tau
        if self.model == "spring":
            acc = (tau - DAMP * self.qd - STIFF * (self.q - self.q_ref)) / MASS
            self.qd += acc * DT
            self.q += self.qd * DT
        else:
            self.step_fairino(tau)
        self.cmd.publish(Float64MultiArray(data=[float(x) for x in self.q]))

    def step_fairino(self, tau):
        x = self.q - self.q_ref
        thr = FAIRINO_THR * max(self.get_parameter("threshold_scale").value, 0.01)
        self.over = np.abs(tau) > thr
        f_eff = np.where(self.over, tau - np.sign(tau) * thr, 0.0)
        # しきい値を超えた関節: 力の向きによける（k=0 なら引き戻しなし）。それ以外: 元の姿勢へ戻る
        v_push = (f_eff - FAIRINO_K * x) / FAIRINO_B
        v_back = -x / max(self.get_parameter("return_time").value, 0.05)
        # 戻る動きは、力がしきい値に近づくほど滑らかに弱める（境目で「よける」と「戻る」が急に切り替わって、カクカクするのを防ぐ）
        back_gain = 1.0 - np.clip(np.abs(tau) / thr, 0.0, 1.0)
        v_cmd = np.clip(np.where(self.over, v_push, back_gain * v_back), -FAIRINO_VMAX, FAIRINO_VMAX)
        # 速度は m/b の時定数で追従し、加速度を最大加速度で制限する
        a = np.clip((v_cmd - self.qd) * FAIRINO_B / FAIRINO_M, -FAIRINO_AMAX, FAIRINO_AMAX)
        self.qd += a * DT
        self.q += self.qd * DT
        # 関節の範囲を超えない（超えたらそこで止める）
        clipped = np.clip(self.q, self.lim[:, 0] + 0.05, self.lim[:, 1] - 0.05)
        self.qd[clipped != self.q] = 0.0
        self.q = clipped

    # ---- ランダムデモ ----
    RANGES = np.array([1.2, 0.6, 0.8, 1.0, 1.0, 1.5])   # 基準姿勢からの、姿勢のばらつきの幅 [rad]

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def settled(self, err, tol=0.01, dwell=1.0):
        """err が tol 以内に入り、そのまま dwell 秒たったら True（到着してから、しばらく止まる）"""
        now = self.now_s()
        if err >= tol:
            self.t_ok = None
        elif self.t_ok is None:
            self.t_ok = now
        return self.t_ok is not None and now - self.t_ok >= dwell

    def next_pose(self):
        """ランダムな姿勢。関節の範囲内で、肘・手首が床（z<0.15m）に潜らないものを選ぶ（random_pose が false なら初期姿勢のまま）"""
        if not self.get_parameter("random_pose").value:
            self.q_ref = np.array(Q_REF)
            self.start_push()   # 姿勢は初期姿勢のまま。すぐ押す
            return
        for _ in range(500):
            q = np.array(Q_REF) + self.rng.uniform(-1, 1, 6) * self.RANGES
            q = np.clip(q, self.lim[:, 0] + 0.1, self.lim[:, 1] - 0.1)
            os, _ = self.kinematics(q)
            if all(o[2] > 0.15 for o in os[2:]):
                break
        self.q_ref = q
        self.rstate, self.rt0, self.t_ok = "move", self.now_s(), None

    def start_push(self):
        """姿勢固定のときは、同じ位置・向きで「小さい力（しきい値未満で動かない）→ 大きい力（動く）」の順に押す。
        組ごとに交互に、向きは「ランダム」と「真横」になる"""
        thr = FAIRINO_THR * max(self.get_parameter("threshold_scale").value, 0.01)
        paired = not self.get_parameter("random_pose").value
        kind = "strong"
        if self.pair is not None:
            seg, frac, d, mag, ratio, mode = self.pair
            self.pair = None
        else:
            self.pair_count += 1
            mode = 1 if (paired and self.pair_count % 2 == 0) else 0   # 偶数番目の組は真横
            for _ in range(300):
                seg = int(self.rng.integers(1, 7))
                frac = float(self.rng.uniform(0.1, 0.9))
                if mode == 1:
                    d = np.array([self.rng.choice([-1.0, 1.0]), 0.0, 0.0])   # 真横（左右どちらか）
                else:
                    d = self.rng.normal(size=3)
                    d /= np.linalg.norm(d)
                mag = float(self.rng.uniform(20, 50))
                p, os, As = self.point(self.q_ref, seg, frac)
                F = self.to_world(d * mag, mode, self.q_ref)
                tau = np.array([np.cross(As[i], p - os[i]) @ F for i in range(seg)])
                ratio = float(np.max(np.abs(tau) / thr[:seg]))
                if ratio > 1.15:   # 大きい力は、しきい値を超える押し方だけ採用
                    break
            if paired:
                self.pair = (seg, frac, d, mag, ratio, mode)
                mag = mag * 0.5 / ratio   # 先に、しきい値の半分の小さい力（動かない）
                kind = "weak"
        p, os, As = self.point(self.q_ref, seg, frac)
        F = self.to_world(d * mag, mode, self.q_ref)
        tau = np.array([np.cross(As[i], p - os[i]) @ F for i in range(seg)])
        self.seg, self.frac, self.force, self.mode = seg, frac, d * mag, mode
        self.push_len = float(self.rng.uniform(2.5, 4.0))
        self.max_disp = 0.0
        self.trial += 1
        self.cur = {"trial": self.trial, "kind": kind, "direction": "side" if mode == 1 else "random", "time": time.strftime("%H:%M:%S"),
                    "q_ref": [round(float(x), 3) for x in self.q_ref], "segment": seg, "position": round(frac, 2),
                    "force_N": [round(float(x), 1) for x in F], "magnitude_N": round(mag, 1), "seconds": round(self.push_len, 1),
                    "joint_torque_Nm": [round(float(x), 1) for x in tau],
                    "torque_over_threshold": round(float(np.max(np.abs(tau) / thr[:seg])), 2)}
        self.get_logger().info(f"試行 {self.trial} ({'弱' if kind == 'weak' else '強'}・{'真横' if mode == 1 else 'ランダム向き'}): 区間{seg} 位置{frac:.2f} 力{np.round(F, 1).tolist()}N ({mag:.0f}N) {self.push_len:.1f}秒")
        self.rstate, self.rt0 = "push", self.now_s()

    def end_push(self):
        self.force = np.zeros(3)
        self.cur["max_joint_displacement_rad"] = round(self.max_disp, 3)
        try:
            with open("/work/logs/random_trials.jsonl", "a") as f:
                f.write(json.dumps(self.cur, ensure_ascii=False) + "\n")
        except OSError:
            pass
        self.q_ref = np.array(Q_REF)   # 押し終わったら、まず初期姿勢へ戻る
        self.rstate, self.rt0, self.t_ok = "home", self.now_s(), None

    def rand_step(self):
        if self.chain is None:
            return
        if not self.get_parameter("random_demo").value:
            if self.rstate != "idle":
                self.rstate, self.force, self.q_ref = "idle", np.zeros(3), np.array(Q_REF)  # 止めたら基準姿勢へ戻る
            return
        now = self.now_s()
        err = float(np.abs(self.q - self.q_ref).max())
        if self.rstate == "idle":
            self.q_ref = np.array(Q_REF)   # 最初も、初期姿勢で delay してから押し始める
            self.rstate, self.rt0, self.t_ok = "home", now, None
        elif self.rstate == "move":
            if self.settled(err, 0.01, 0.5) or now - self.rt0 > 20:
                self.start_push()
        elif self.rstate == "push":
            self.last = self.get_clock().now()
            self.max_disp = max(self.max_disp, err)
            if now - self.rt0 > self.push_len:
                self.end_push()
        elif self.rstate == "home":   # 初期姿勢に着いてから、次のランダムな姿勢へ動く
            if self.settled(err, 0.01, self.get_parameter("delay_before_push").value) or now - self.rt0 > 25:
                self.next_pose()

    def draw_status(self):
        """ランダムデモ中の段階を、画面の上に文字で出す。
        HOME（緑）= 初期姿勢に着いて止まっている / RETURNING（黄）= 初期姿勢へ戻っている / PUSH（赤）= 力をかけている"""
        if not self.get_parameter("random_demo").value:
            return
        text = {"push": ("PUSH", (1.0, 0.4, 0.4)),
                "home": ("HOME", (0.3, 1.0, 0.3)) if self.t_ok is not None else ("RETURNING", (1.0, 0.8, 0.2))}.get(self.rstate)
        if text is None:
            return
        m = Marker()
        m.header.frame_id = "base_link"
        m.ns, m.id, m.type, m.action = "status", 0, Marker.TEXT_VIEW_FACING, Marker.ADD
        m.pose.position = Point(x=-0.2, y=0.0, z=1.0)
        m.scale.z = 0.12
        m.color.r, m.color.g, m.color.b, m.color.a = (*text[1], 1.0)
        m.text = text[0]
        m.lifetime.nanosec = 400_000_000   # 出し続けないと勝手に消える
        self.status.publish(m)

    def draw(self):
        self.draw_status()
        if self.chain is None:
            return
        if not self.active():
            if self.shown:
                m = Marker()
                m.action = Marker.DELETEALL
                self.mk.publish(MarkerArray(markers=[m]))
                self.shown = False
            return
        p, _, _ = self.point(self.q, self.seg, self.frac)
        end = p + self.world_force() * FORCE_SCALE
        a = Marker()
        a.header.frame_id = "base_link"
        a.ns, a.id, a.type, a.action = "force", 0, Marker.ARROW, Marker.ADD
        a.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])), Point(x=float(end[0]), y=float(end[1]), z=float(end[2]))]
        a.scale.x, a.scale.y, a.scale.z = 0.012, 0.025, 0.04
        a.color.r, a.color.g, a.color.b, a.color.a = 1.0, 0.1, 0.1, 1.0
        # 文字は、力をかけた位置のそばに出す（RViz の文字は日本語が崩れるので、英数字だけ）
        lines = [(f"{np.linalg.norm(self.force):.0f}N", (1.0, 0.9, 0.2))]
        texts = []
        for k, (txt, rgb) in enumerate(lines):
            t = Marker()
            t.header.frame_id = "base_link"
            t.ns, t.id, t.type, t.action = "label", k, Marker.TEXT_VIEW_FACING, Marker.ADD
            t.pose.position = Point(x=float(p[0]), y=float(p[1]), z=float(p[2] + 0.10 + 0.10 * (len(lines) - 1 - k)))
            t.scale.z = 0.09
            t.color.r, t.color.g, t.color.b, t.color.a = (*rgb, 1.0)
            t.text = txt
            t.lifetime.nanosec = 400_000_000   # 押すのをやめると勝手に消える
            texts.append(t)
        a.lifetime.nanosec = 400_000_000
        self.mk.publish(MarkerArray(markers=[a] + texts))
        self.shown = True


def main():
    rclpy.init()
    rclpy.spin(LinkCompliance())


if __name__ == "__main__":
    main()
