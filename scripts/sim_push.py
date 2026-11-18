#!/usr/bin/env python3
"""手首の先を押したとき、実機の関節インピーダンス制御で腕がどう動くかを計算して保存する(シミュレーションのみ。実機には何も送らない)。

  python3 scripts/sim_push.py                      # 既定: 初期姿勢から -Y に 10〜100 N、2 秒押して離す
  python3 scripts/sim_push.py --dir 0 -1 0 --forces 20 40 60 --push 2 --release 1

モデル(マニュアル 9.33.2.2 関節空間の説明と、2026-11-11 の実機の試しに合わせたもの):
- 押す力 F [N](base_link 座標系)を、関節トルク τ = Jᵀ F に直す。押す点は手首の先(フランジ)。
- |τ| がしきい値を超えた関節だけが、力の向きに逃げる。逃げる速さは (|τ| - しきい値) / b を、最大速度・最大加速度で制限する。
  b = 0.1 では少しでも超えるとすぐ最大速度になるので、実際には「超えたら最大速度で逃げる」動きになる。
- 力がしきい値を下回る(離す)と止まり、元の位置には戻らない(実機の試しで戻らなかったため)。
- しきい値は 2 通りで計算する: マニュアルどおり(nominal)と、実機の試しに合わせたもの(observed: j1 が約 22 Nm で動き出した → 2.2 倍)。
m・b の単位や内部の計算はわからないので、動き出す力の目安としては使えるが、動きの細かいところは実機と一致しない。
"""
import argparse
import csv
import json
import math
import os
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

# gui/fr3_core.py の CHAIN と同じ(URDF fairino3_v6)。fr3_core を import すると GUI のログが作られるので、写して使う
CHAIN = [((0, 0, 0), (0, 0, 0)), ((0, 0, 0.14), (math.pi / 2, 0, 0)), ((-0.28, 0, 0), (0, 0, 0)),
         ((-0.24001, 0, 0), (0, 0, 0)), ((0, 0, 0.102), (math.pi / 2, 0, 0)), ((0, 0, 0.102), (-math.pi / 2, 0, 0))]
HOME_DEG = [-10.93, -93.27, 77.22, -73.96, -90.0, -100.94]  # 初期姿勢(init_pose.csv)

# GUI の既定値(関節空間)と同じ
THR = np.array([10.0, 10.0, 10.0, 1.0, 1.0, 1.0])  # しきい値 [Nm](マニュアルの下限)
M = np.array([0.04, 0.04, 0.04, 0.01, 0.01, 0.01])
B = np.array([0.1, 0.1, 0.1, 0.08, 0.08, 0.08])
VMAX = math.radians(10.0)  # 最大速度 10 deg/s(GUI のゆっくりした既定値)
AMAX = math.radians(10.0)  # 最大加速度 10 deg/s²
OBSERVED_SCALE = 2.2  # 実機では、j1 の表示トルクが約 22 Nm(しきい値 10 Nm の 2.2 倍)で動き出した
DT = 0.01


def rot(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ np.array(
        [[1, 0, 0], [0, cr, -sr], [0, sr, cr]])


def tip(q):
    """手首の先(フランジ)の位置 [m]"""
    R, p = np.eye(3), np.zeros(3)
    for (xyz, rpy), qi in zip(CHAIN, q):
        p = p + R @ np.array(xyz)
        R = R @ rot(*rpy) @ np.array([[math.cos(qi), -math.sin(qi), 0], [math.sin(qi), math.cos(qi), 0], [0, 0, 1]])
    return p


def jac(q, h=1e-6):
    p0, J = tip(q), np.zeros((3, 6))
    for i in range(6):
        dq = q.copy()
        dq[i] += h
        J[:, i] = (tip(dq) - p0) / h
    return J


def simulate(F_dir, F_mag, thr, t_push, t_release):
    q, qd = np.radians(HOME_DEG), np.zeros(6)
    F = np.asarray(F_dir, float) / np.linalg.norm(F_dir) * F_mag
    rows, moved_at, tau_peak = [], None, np.zeros(6)
    for k in range(int((t_push + t_release) / DT) + 1):
        t = k * DT
        tau = jac(q).T @ F if t < t_push else np.zeros(6)
        tau_peak = np.maximum(tau_peak, np.abs(tau))
        over = np.abs(tau) > thr
        v_cmd = np.where(over, np.clip((tau - np.sign(tau) * thr) / B, -VMAX, VMAX), 0.0)
        a = np.clip((v_cmd - qd) * B / M, -AMAX, AMAX)
        qd = qd + a * DT
        q = q + qd * DT
        if moved_at is None and np.max(np.abs(np.degrees(q) - HOME_DEG)) > 0.1:
            moved_at = t
        rows.append([t, F_mag, *np.degrees(q), *(tip(q) * 1000), *tau])
    return rows, moved_at, tau_peak


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", nargs=3, type=float, default=[0, -1, 0], help="押す向き(base_link 座標系)。既定 -Y")
    ap.add_argument("--forces", nargs="+", type=float, default=[10, 20, 30, 40, 50, 60, 70, 80, 100], help="押す力 [N]")
    ap.add_argument("--push", type=float, default=2.0, help="押し続ける時間 [s]")
    ap.add_argument("--release", type=float, default=1.0, help="離したあとに計算する時間 [s]")
    ap.add_argument("--out", default=None, help="保存先(既定: logs/sim_push_日時/)")
    a = ap.parse_args()
    out = a.out or os.path.join(ROOT, "logs", time.strftime("sim_push_%Y%m%d_%H%M%S"))
    os.makedirs(out, exist_ok=True)

    q0 = np.radians(HOME_DEG)
    J0, Fd = jac(q0), np.asarray(a.dir, float) / np.linalg.norm(a.dir)
    tau_per_N = J0.T @ Fd  # 1 N 押したときの各関節トルク [Nm/N](= 腕の長さのようなもの)
    summary = {"pose_deg": HOME_DEG, "direction": Fd.tolist(), "push_s": a.push, "release_s": a.release,
               "vmax_deg_s": math.degrees(VMAX), "amax_deg_s2": math.degrees(AMAX),
               "tau_per_newton": tau_per_N.round(4).tolist(), "cases": {}}
    header = ["t_s", "force_N"] + [f"j{i}_deg" for i in range(1, 7)] + ["x_mm", "y_mm", "z_mm"] + [f"tau{i}_Nm" for i in range(1, 7)]
    for case, scale in (("nominal", 1.0), ("observed", OBSERVED_SCALE)):
        thr = THR * scale
        # 動き出す最小の力: どれかの関節で |τ| > しきい値 になる力
        with np.errstate(divide="ignore"):
            f_need = np.where(np.abs(tau_per_N) > 1e-6, thr / np.abs(tau_per_N), np.inf)
        res = []
        with open(os.path.join(out, f"traj_{case}.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(header)
            for Fm in a.forces:
                rows, moved_at, peak = simulate(Fd, Fm, thr, a.push, a.release)
                w.writerows([[round(v, 5) for v in r] for r in rows])
                end = np.array(rows[-1])
                dq = end[2:8] - HOME_DEG
                res.append({"force_N": Fm, "moves": moved_at is not None, "moved_at_s": moved_at,
                            "joint_change_deg": dq.round(2).tolist(),
                            "tip_move_mm": (end[8:11] - np.array(rows[0][8:11])).round(1).tolist(),
                            "tau_peak_Nm": peak.round(2).tolist()})
        summary["cases"][case] = {"threshold_Nm": thr.tolist(), "min_force_to_move_N": float(np.min(f_need)),
                                  "min_force_per_joint_N": [None if not np.isfinite(v) else round(float(v), 1) for v in f_need],
                                  "results": res}
    json.dump(summary, open(os.path.join(out, "summary.json"), "w"), ensure_ascii=False, indent=2)
    plot(summary, out)
    # 画面にまとめを出す
    print(f"保存先: {out}")
    print(f"押す向き {Fd.round(2).tolist()}、1 N あたりの関節トルク [Nm/N]: {tau_per_N.round(3).tolist()}")
    for case, c in summary["cases"].items():
        print(f"\n[{case}] しきい値 {c['threshold_Nm']} → 動き出す最小の力 約 {c['min_force_to_move_N']:.0f} N "
              f"(約 {c['min_force_to_move_N'] / 9.8:.1f} kg重)")
        for r in c["results"]:
            print(f"  {r['force_N']:5.0f} N: " + ("動く  " if r["moves"] else "動かない") +
                  f"  j1 {r['joint_change_deg'][0]:+6.2f}°  手先の移動 {r['tip_move_mm']} mm")


def plot(summary, out):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib import font_manager
        for fp in ("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",):
            if os.path.exists(fp):
                font_manager.fontManager.addfont(fp)
                plt.rcParams["font.family"] = font_manager.FontProperties(fname=fp).get_name()
    except ImportError:
        return
    fig, axs = plt.subplots(1, 2, figsize=(11, 4.2))
    for case, style in (("nominal", "-o"), ("observed", "--s")):
        c = summary["cases"][case]
        F = [r["force_N"] for r in c["results"]]
        d = [math.hypot(*r["tip_move_mm"][:2]) for r in c["results"]]
        axs[0].plot(F, d, style, label=f"{case}(動き出す力 約 {c['min_force_to_move_N']:.0f} N)")
        axs[0].axvline(c["min_force_to_move_N"], ls=":", color="gray")
    axs[0].set(xlabel="押す力 [N]", ylabel=f"手先の水平の移動 [mm]({summary['push_s']:g} 秒押す)", title="力と動く量")
    axs[0].legend(fontsize=8)
    axs[0].grid(alpha=0.3)
    tau = summary["tau_per_newton"]
    axs[1].bar([f"j{i}" for i in range(1, 7)], [abs(v) * 30 for v in tau], color="#888", label="30 N で押したときの |τ|")
    axs[1].plot([f"j{i}" for i in range(1, 7)], summary["cases"]["nominal"]["threshold_Nm"], "r_", ms=30, mew=3, label="しきい値(マニュアル)")
    axs[1].set(ylabel="Nm", title="30 N で押したときの関節トルクとしきい値")
    axs[1].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "summary.png"), dpi=110)


if __name__ == "__main__":
    main()
