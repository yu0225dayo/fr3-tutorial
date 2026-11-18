"""FR3 操作 GUI(Tk 版)。コンテナ内で、RViz の隣に並べて表示する(docker/run.sh gui)。

腕の関節角は /joint_states に流し、RViz で実際の腕の動きを見る(シミュレーションも実機も同じ)。
禁止エリアと目標位置は /gui_markers に出す。安全ルールは fr3_core.py(ドライラン既定・範囲検査・禁止エリア・ログ)。
"""
import argparse
import signal
import math
import time
import tkinter as tk
import tkinter.font as tkfont
from tkinter import messagebox, ttk

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from interactive_markers import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker, MarkerArray

import fr3_core as core

C = ["x", "y", "z", "rx", "ry", "rz"]
AXC = [(1.0, 0.1, 0.1), (0.1, 0.8, 0.1), (0.2, 0.3, 1.0)]  # x=赤 y=緑 z=青(RViz の慣例)


def quat_from_R(R):
    """回転行列 → (x, y, z, w)"""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, s / 4)
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(R[i, i] - R[j, j] - R[k, k] + 1) * 2
    q = [0.0] * 4
    q[i] = s / 4
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return tuple(q)


def R_from_quat(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def arrow(ns, i, frame, p0, p1, color, d=0.012, a=1.0):
    m = Marker()
    m.header.frame_id, m.ns, m.id, m.type = frame, ns, i, Marker.ARROW
    m.points = [Point(x=float(p0[0]), y=float(p0[1]), z=float(p0[2])), Point(x=float(p1[0]), y=float(p1[1]), z=float(p1[2]))]
    m.scale.x, m.scale.y, m.scale.z = d, d * 2, d * 3
    m.color.r, m.color.g, m.color.b, m.color.a = (*color, a)
    return m


def text(ns, i, pos, label, color, size=0.07):
    m = Marker()
    m.header.frame_id, m.ns, m.id, m.type, m.text = "base_link", ns, i, Marker.TEXT_VIEW_FACING, label
    m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, pos)
    m.pose.orientation.w = 1.0
    m.scale.z = size
    m.color.r, m.color.g, m.color.b, m.color.a = (*color, 1.0)
    return m


class Gui:
    def __init__(self, root, execute):
        self.root, self.A = root, core.App(execute)
        rclpy.init()
        self.node = rclpy.create_node("fr3_gui")
        self.pub_js = self.node.create_publisher(JointState, "/joint_states", 10)
        self.pub_home = self.node.create_publisher(JointState, "/home/joint_states", 10)  # 比較用の初期姿勢の腕
        self.pub_arm = self.node.create_publisher(MarkerArray, "/gui_arm", 10)  # 腕の形(透明度を切り替えるため自前で描く)
        self.arm_mode, self.arm_t = None, 0.0
        self.pub_mk = self.node.create_publisher(MarkerArray, "/gui_markers", 10)
        self.S = {}
        self.busy = False
        self.drag_t = 0.0
        self.marker_pose = None
        self.ims = InteractiveMarkerServer(self.node, "fr3_target")
        root.title("FR3 Control")
        f = tkfont.nametofont("TkDefaultFont")
        f.configure(family="Noto Sans CJK JP", size=10)
        tkfont.nametofont("TkTextFont").configure(family="Noto Sans CJK JP", size=10)
        self.build()
        self.load_zone_rows()
        self.make_target()
        root.after(800, self.connect)  # シミュレーションには、起動時に自動で接続する(実機は手動)
        self.spin()
        self.tick()
        self.slow_tick()

    # ------------------------------------------------------------ 画面
    def build(self):
        r = self.root
        top = ttk.Frame(r, padding=6)
        top.pack(fill="x")
        # 各行を別の枠にする(1 つの grid にすると、長い行(状態・モードの行)に引っぱられて列が広がり、「終了」が右で見切れた)
        self.be = tk.StringVar(value="sim")
        row0 = ttk.Frame(top)
        row0.pack(fill="x")
        ttk.Radiobutton(row0, text="シミュレーション", variable=self.be, value="sim").pack(side="left")
        ttk.Radiobutton(row0, text="実機", variable=self.be, value="real").pack(side="left", padx=(4, 0))
        self.ip = tk.StringVar(value="192.168.58.2")
        ttk.Entry(row0, textvariable=self.ip, width=13).pack(side="left", padx=4)
        ttk.Button(row0, text="終了", width=5, command=self.ask_quit).pack(side="right", padx=1)
        ttk.Button(row0, text="切断", width=5, command=self.disconnect).pack(side="right", padx=1)
        ttk.Button(row0, text="接続", width=5, command=self.connect).pack(side="right", padx=1)
        self.lbl_state = tk.Label(top, text="未接続", fg="gray", anchor="w", justify="left", wraplength=580)
        self.lbl_state.pack(fill="x")
        self.safety = tk.BooleanVar()
        ttk.Checkbutton(top, text="非常停止の場所を確認し、周囲に人・物がない(実機で必要)", variable=self.safety,
                        command=lambda: setattr(self.A, "safety_ok", self.safety.get())).pack(anchor="w")
        mf = ttk.Frame(top)
        mf.pack(fill="x", pady=2)
        ttk.Label(mf, text="モード:").pack(side="left")
        ttk.Button(mf, text="手動モードにする", command=lambda: self.set_mode(True)).pack(side="left", padx=2)
        ttk.Button(mf, text="自動モードにする", command=lambda: self.set_mode(False)).pack(side="left", padx=2)
        ttk.Button(mf, text="エラーをリセット", command=self.reset_error).pack(side="right", padx=2)
        self.btn_stop = tk.Button(r, text="■ 停止", bg="#c62828", fg="white", font=("Noto Sans CJK JP", 16, "bold"), command=self.stop)
        self.btn_stop.pack(fill="x", padx=6, pady=3)
        # インピーダンスを入れたままの手順の、いまの段と、インピーダンスが切れるまでのカウントダウン(どのタブでも見える)
        self.lbl_count = tk.Label(r, text="", anchor="w", justify="left", font=("Noto Sans CJK JP", 13, "bold"), fg="white", bg="#6a1b9a", padx=6)
        # ここから下は、縦にスクロールできる領域(接続の欄と「■ 停止」は、スクロールしても常に見えるよう外に置く)
        r = self.scroll_area(r)
        self.btn_home = tk.Button(r, text="⌂ 初期姿勢へ戻る", bg="#2e7d32", fg="white", font=("Noto Sans CJK JP", 13, "bold"),
                                  disabledforeground="#cfd8dc", command=self.go_home)
        self.btn_home.pack(fill="x", padx=6, pady=2)
        self.msg = tk.Label(r, text="接続先を選んで「接続」を押してください。", anchor="w", justify="left", wraplength=560)
        self.msg.pack(fill="x", padx=6)
        self.lbl_pose = tk.Label(r, text="", anchor="w", justify="left", font=("DejaVu Sans Mono", 9))
        self.lbl_pose.pack(fill="x", padx=6)

        nb = ttk.Notebook(r)
        nb.pack(fill="both", expand=True, padx=6, pady=4)
        self.nb = nb
        self.drag_labels = []  # ドラッグの状態・手先の位置を出すラベル(ドラッグのタブと禁止エリアのタブ)
        self.build_move(nb)
        self.build_drag(nb)
        self.build_zones(nb)
        self.build_imp(nb)

    def scroll_area(self, parent):
        """縦のスクロールバー付きの領域を作り、中身を置くフレームを返す。マウスのホイールでも動かせる"""
        box = ttk.Frame(parent)
        box.pack(fill="both", expand=True)
        cv = tk.Canvas(box, highlightthickness=0)
        sb = ttk.Scrollbar(box, orient="vertical", command=cv.yview)
        cv.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        cv.pack(side="left", fill="both", expand=True)
        inner = ttk.Frame(cv)
        win = cv.create_window((0, 0), window=inner, anchor="nw")
        inner.bind("<Configure>", lambda e: cv.configure(scrollregion=cv.bbox("all")))
        cv.bind("<Configure>", lambda e: cv.itemconfigure(win, width=e.width))  # 中身の幅を、窓の幅に合わせる

        def wheel(e):
            # 禁止エリアの一覧など、自分でスクロールする部品の上では、そちらに任せる
            if isinstance(e.widget, (ttk.Treeview, tk.Listbox, tk.Text)):
                return
            if str(e.widget).startswith(str(cv)):
                cv.yview_scroll(-1 if (e.num == 4 or getattr(e, "delta", 0) > 0) else 1, "units")
        for ev in ("<Button-4>", "<Button-5>", "<MouseWheel>"):
            self.root.bind_all(ev, wheel, add="+")
        return inner

    def entries(self, parent, names, vals, row0=0, width=8):
        out = []
        for i, n in enumerate(names):
            ttk.Label(parent, text=n).grid(row=row0 + i // 3, column=(i % 3) * 2, sticky="e", padx=2, pady=1)
            v = tk.StringVar(value=str(vals[i]))
            ttk.Entry(parent, textvariable=v, width=width).grid(row=row0 + i // 3, column=(i % 3) * 2 + 1, padx=2)
            out.append(v)
        return out

    def build_move(self, nb):
        p = ttk.Frame(nb, padding=6)
        nb.add(p, text="移動")
        tk.Label(p, anchor="w", justify="left", bg="#fff8dc", fg="#333", padx=6, pady=4, wraplength=540,
                 text="使い方\n① 上の「接続」を押す(シミュレーションは自動で接続済み)\n② 目標を決める: 下の ± ボタンで動かす / 数字を入れる / RViz の黄色い球の矢印をドラッグ\n"
                      "③ 一番下の大きな「▶ 目標へ移動」を押す(押すまで腕は動きません)\n止めたいときは、赤い「■ 停止」").pack(fill="x")
        ttk.Label(p, text="② 目標の位置(ベース座標系, x y z は mm・rx ry rz は deg)").pack(anchor="w", pady=(8, 0))
        pf = ttk.Frame(p)
        pf.pack(anchor="w")
        p0, R0 = core.fk(np.radians(core.HOME_DEG))
        self.tcp = self.entries(pf, C, [round(v, 1) for v in core.pose_from(p0, R0)], 0)
        row = ttk.Frame(p)  # 移動(左)と回転(右)を、横に並べる
        row.pack(anchor="w", pady=3)
        nf = ttk.Frame(row)
        nf.pack(side="left", anchor="n")
        ttk.Label(nf, text="目標を動かす(1回 mm)").grid(row=0, column=0, columnspan=2)
        self.step = tk.StringVar(value="20")
        ttk.Entry(nf, textvariable=self.step, width=5).grid(row=0, column=2)
        for i, (a, neg, pos) in enumerate((("x", "x−", "x＋"), ("y", "y−", "y＋"), ("z", "z− (下)", "z＋ (上)"))):
            ttk.Button(nf, text=neg, width=7, command=lambda i=i: self.nudge(i, -1)).grid(row=1 + i, column=0, columnspan=2, pady=1)
            ttk.Button(nf, text=pos, width=7, command=lambda i=i: self.nudge(i, 1)).grid(row=1 + i, column=2, columnspan=2, pady=1)
        rf = ttk.Frame(row)
        rf.pack(side="left", anchor="n", padx=(16, 0))
        ttk.Label(rf, text="目標の向きを回す(1回 deg)").grid(row=0, column=0, columnspan=2)
        self.rstep = tk.StringVar(value="10")
        ttk.Entry(rf, textvariable=self.rstep, width=5).grid(row=0, column=2)
        for i, a in enumerate(("rx", "ry", "rz")):
            ttk.Button(rf, text=f"{a}−", width=7, command=lambda i=i: self.turn(i, -1)).grid(row=1 + i, column=0, columnspan=2, pady=1)
            ttk.Button(rf, text=f"{a}＋", width=7, command=lambda i=i: self.turn(i, 1)).grid(row=1 + i, column=2, columnspan=2, pady=1)
        ttk.Button(p, text="目標を、いまの手先の位置に戻す", command=self.use_current).pack(anchor="w", pady=2)
        tk.Label(p, anchor="w", fg="gray", wraplength=540, justify="left",
                 text="(x＋ は RViz の赤い +X 軸の向き、y＋ は緑の +Y 軸の向きです。初期姿勢の腕は、-X 側に伸びています)").pack(fill="x")
        self.show_zones = tk.BooleanVar(value=True)  # 「移動」「ドラッグ」タブで共通
        tk.Checkbutton(p, text="RViz に禁止エリアを表示(外しても、判定と止める働きはそのまま)", variable=self.show_zones, selectcolor="white").pack(anchor="w")
        self.axis_filter(p).pack(anchor="w", padx=20)
        vf = ttk.Frame(p)
        vf.pack(anchor="w", pady=4)
        ttk.Label(vf, text=f"速度 %(上限 {core.LIM['max_vel_percent']})").grid(row=0, column=0)
        self.vel = tk.StringVar(value=str(core.LIM["default_vel_percent"]))
        ttk.Entry(vf, textvariable=self.vel, width=5).grid(row=0, column=1, padx=4)
        # インピーダンスを入れたまま移動する(コントローラのインピーダンスは、軌道があるときだけ効くため)
        self.move_imp = tk.BooleanVar(value=False)
        tk.Checkbutton(p, text="インピーダンスを入れて移動する(値は「インピーダンス」タブ)\n"
                       "  開始 → 1 秒後に移動 → 着いて 5 秒後に終了。動いている最中に押すと、腕が譲る\n"
                       "  着いたあと押されてずれたら、目標へ送り直し、終了後も目標に合わせる",
                       variable=self.move_imp, selectcolor="white", justify="left").pack(anchor="w")
        self.lbl_seq = tk.Label(p, text="", anchor="w", justify="left", fg="#6a1b9a")
        self.lbl_seq.pack(fill="x")
        self.lbl_preview = tk.Label(p, text="", anchor="w", justify="left", wraplength=540)
        self.lbl_preview.pack(fill="x")
        tk.Button(p, text="▶ 目標へ移動", bg="#1f5fbf", fg="white", font=("Noto Sans CJK JP", 15, "bold"),
                  command=lambda: self.move("tcp")).pack(fill="x", pady=6)
        ttk.Separator(p).pack(fill="x", pady=4)
        ttk.Label(p, text="(別の方法) 関節角度で指定 deg").pack(anchor="w")
        jf = ttk.Frame(p)
        jf.pack(anchor="w")
        self.jnt = self.entries(jf, [f"j{i}" for i in range(1, 7)], core.HOME_DEG, 0)
        jb = ttk.Frame(p)
        jb.pack(anchor="w", pady=2)
        ttk.Button(jb, text="いまの関節角を入れる", command=self.use_current_joints).grid(row=0, column=0, padx=2)
        ttk.Button(jb, text="初期姿勢を入れる", command=lambda: [v.set(str(h)) for v, h in zip(self.jnt, core.HOME_DEG)]).grid(row=0, column=1, padx=2)
        ttk.Button(jb, text="関節角度へ移動", command=lambda: self.move("joint")).grid(row=0, column=2, padx=2)

    def axis_filter(self, parent):
        """表示する禁止エリアを、面の軸ごとに選ぶチェック(どのタブに置いても同じ変数)"""
        if not hasattr(self, "show_axis"):
            self.show_axis = {k: tk.BooleanVar(value=True) for k in ("x", "y", "z", "box")}
        f = ttk.Frame(parent)
        ttk.Label(f, text="表示する面:").pack(side="left")
        for k, lab, col in (("x", "x の面", "#d32f2f"), ("y", "y の面", "#2e7d32"), ("z", "z の面", "#1e40ff"), ("box", "箱", "#ef8a00")):
            tk.Checkbutton(f, text=lab, variable=self.show_axis[k], fg=col, activeforeground=col, selectcolor="white").pack(side="left", padx=2)
        return f

    def zone_tab(self):
        """「禁止エリア」タブを開いているか"""
        try:
            return self.nb.tab(self.nb.select(), "text") == "禁止エリア"
        except tk.TclError:
            return False

    def zone_mode(self):
        """エリアを作る場面か(「ドラッグ」「禁止エリア」タブを開いている、またはドラッグ中)"""
        try:
            tab = self.nb.tab(self.nb.select(), "text")
        except tk.TclError:
            tab = ""
        return tab in ("ドラッグ", "禁止エリア") or self.A.drag_on

    def publish_arm(self, force=False):
        """腕の形を RViz に描く。移動のとき: 今の腕は不透明、初期姿勢の腕は半透明。
        エリアを作るとき: 今の腕は半透明、初期姿勢の腕は出さず、判定に使う点(壁ができる位置)を赤紫の球で出す。
        マーカーはリンクの座標に貼り付ける(frame_locked)ので、腕が動いてもなめらかについてくる"""
        mode = "zone" if self.zone_mode() else "move"
        now = time.time()
        if not force and mode == self.arm_mode and now - self.arm_t < 2.0:
            return
        if mode != self.arm_mode and self.arm_mode is not None:
            # エリアを作るときは、移動の目標マーカー(黄色い球と矢印)を隠して、判定に使う点を見やすくする
            if mode == "zone":
                self.ims.erase("target")
                self.ims.applyChanges()
            else:
                self.make_target()
        self.arm_mode, self.arm_t = mode, now
        arr, stamp = MarkerArray(), self.node.get_clock().now().to_msg()

        def mesh(ns, i, frame, alpha):
            m = Marker()
            m.header.frame_id, m.header.stamp, m.ns, m.id = frame, stamp, ns, i
            if alpha <= 0:
                m.action = Marker.DELETE
                return m
            m.type, m.mesh_resource, m.frame_locked = Marker.MESH_RESOURCE, ARM_MESH.format(ARM_LINKS[i]), True
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 1.0
            m.color.r, m.color.g, m.color.b, m.color.a = (*ARM_RGB, alpha)
            return m
        for i, ln in enumerate(ARM_LINKS):
            arr.markers.append(mesh("arm_now", i, ln, 0.35 if mode == "zone" else 1.0))
            arr.markers.append(mesh("arm_home", i, "home/" + ln, 0.0 if mode == "zone" else 0.3))
        # 判定に使う点: フランジ(wrist3_link の原点)から、ツールの向き(wrist3_link の z)に tool_length_mm 先
        tl = core.LIM.get("tool_length_mm", 100) / 1000
        for i, (typ, pos, sc, txt) in enumerate(((Marker.SPHERE, (0, 0, tl), 0.04, ""), (Marker.TEXT_VIEW_FACING, (0, 0, tl + 0.06), 0.04, "CHECK POINT"))):
            m = Marker()
            m.header.frame_id, m.header.stamp, m.ns, m.id = "wrist3_link", stamp, "check_point", i
            if mode != "zone":
                m.action = Marker.DELETE
            else:
                m.type, m.frame_locked, m.text = typ, True, txt
                m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, pos)
                m.pose.orientation.w = 1.0
                m.scale.x = m.scale.y = m.scale.z = sc
                m.color.r, m.color.g, m.color.b, m.color.a = 0.85, 0.1, 0.85, 1.0
            arr.markers.append(m)
        self.pub_arm.publish(arr)

    def drag_buttons(self, parent):
        """ドラッグ開始・終了のボタンと、状態のラベル(ドラッグのタブ・禁止エリアのタブで共通)"""
        f = ttk.Frame(parent)
        tk.Button(f, text="✋ ドラッグ開始", bg="#e0f0ff", font=("Noto Sans CJK JP", 12, "bold"),
                  command=lambda: self.drag(True)).pack(side="left", padx=2)
        tk.Button(f, text="ドラッグ終了(その場で止める)", font=("Noto Sans CJK JP", 12),
                  command=lambda: self.drag(False)).pack(side="left", padx=2)
        lbl = tk.Label(parent, text="ドラッグ OFF", anchor="w", justify="left", font=("DejaVu Sans Mono", 10))
        self.drag_labels.append(lbl)
        return f, lbl

    def build_drag(self, nb):
        p = ttk.Frame(nb, padding=6)
        nb.add(p, text="ドラッグ")
        tk.Label(p, anchor="w", justify="left", bg="#fff8dc", fg="#333", padx=6, pady=4, wraplength=540,
                 text="手で腕を持って動かすモード。腕の重さを打ち消すので、軽い力で動き、手を離すとその場にとどまる。\n"
                      "① 実機に接続し、上の安全確認にチェック\n② 「ドラッグ開始」\n③ まず手を添えるだけで、勝手に動かないか見る\n"
                      "④ 手首の先を持ってゆっくり動かす\n⑤ 「ドラッグ終了」か「■ 停止」で止める").pack(fill="x")
        f, lbl = self.drag_buttons(p)
        f.pack(anchor="w", pady=6)
        lbl.pack(fill="x")
        tk.Checkbutton(p, text="RViz に禁止エリアを表示(外しても、判定と止める働きはそのまま)", variable=self.show_zones, selectcolor="white").pack(anchor="w", pady=(4, 0))
        self.axis_filter(p).pack(anchor="w", padx=20)
        tk.Label(p, anchor="w", justify="left", wraplength=540, fg="#555",
                 text="・先端の負荷(質量・重心)の設定が実際と合っていないと、腕が勝手に下がる。\n"
                      "・手先が禁止エリアに入ったら、自動でドラッグモードを抜けて止まる。\n"
                      "・ドラッグ中は、移動とインピーダンス制御は受け付けない。\n"
                      "・禁止エリアを手先の位置で決めるときは、「禁止エリア」タブでもドラッグを開始・終了できる。").pack(fill="x", pady=6)

    def build_zones(self, nb):
        p = ttk.Frame(nb, padding=6)
        nb.add(p, text="禁止エリア")
        tk.Label(p, anchor="w", justify="left", bg="#fff8dc", fg="#333", padx=6, pady=4, wraplength=540,
                 text="使い方\n禁止エリア = 手先や腕を入れたくない場所。入る移動は手前で止め、動作中・ドラッグ中に入ったら止める。\n"
                      "■ 壁を 1 点で作る(机の面・壁など)\n"
                      "  ① 実機に接続し、上の安全確認にチェック → 「✋ ドラッグ開始」\n"
                      "  ② 手先を、壁にしたい位置へ手で動かす → 「ドラッグ終了」\n"
                      "  ③ 軸(x・y・z)と、禁止する側(＋側・−側)を選ぶ(例: 机の面なら z の −側)\n"
                      "  ④ 「この位置に壁を作る(保存)」→ 確認して OK。すぐ効き、RViz に赤い壁が出る\n"
                      "■ 物の周りを箱で囲む: 角を 2 か所以上「① いまの手先を記録」→ 「② 囲む箱を入力欄に入れる」→ 「追加 / 更新」\n"
                      "■ 数字で作る・直す: 一覧から選ぶと下の欄に入る → 名前・min・max を書いて「追加 / 更新」。消すのは「削除」\n"
                      "※ 座標は mm、ベース座標系(RViz の BASE の矢印)。保存先は config/zones.json(書き換えるとすぐ反映)").pack(fill="x", pady=(0, 4))
        cols = ("on", "name", "xmin", "xmax", "ymin", "ymax", "zmin", "zmax")
        self.tree = ttk.Treeview(p, columns=cols, show="headings", height=5)
        for c, w in zip(cols, (30, 90, 55, 55, 55, 55, 55, 55)):
            self.tree.heading(c, text=c)
            self.tree.column(c, width=w, anchor="center")
        self.tree.pack(fill="x", pady=4)
        self.tree.bind("<<TreeviewSelect>>", self.zone_select)
        ef = ttk.Frame(p)
        ef.pack(anchor="w")
        # 1 段目: 名前、2 段目: xmin ymin zmin、3 段目: xmax ymax zmax。self.zv の並びは (名前, xmin, xmax, ymin, ymax, zmin, zmax)
        self.zv = [tk.StringVar(value=v) for v in ("zone1", "-800", "-400", "-400", "400", "0", "900")]
        ttk.Label(ef, text="名前").grid(row=0, column=0, sticky="e", padx=2, pady=2)
        ttk.Entry(ef, textvariable=self.zv[0], width=32).grid(row=0, column=1, columnspan=5, sticky="w", padx=2)
        for r, (lab, idx) in enumerate((("min", (1, 3, 5)), ("max", (2, 4, 6))), start=1):
            for c, (ax, i) in enumerate(zip("xyz", idx)):
                ttk.Label(ef, text=f"{ax}{lab}").grid(row=r, column=c * 2, sticky="e", padx=(8 if c else 2, 2), pady=2)
                ttk.Entry(ef, textvariable=self.zv[i], width=10).grid(row=r, column=c * 2 + 1, sticky="w")
        self.zon = tk.BooleanVar(value=True)
        ttk.Checkbutton(ef, text="有効", variable=self.zon).grid(row=3, column=0, columnspan=2, sticky="w", pady=2)
        self.ztcp = tk.BooleanVar(value=True)
        ttk.Checkbutton(ef, text="手先だけで判定", variable=self.ztcp).grid(row=3, column=2, columnspan=4, sticky="w")
        # ドラッグモードなどで手先を動かし、その位置から禁止エリアを決める
        gf = ttk.LabelFrame(p, text="手先の位置から決める(ドラッグで動かして)", padding=4)
        gf.pack(fill="x", pady=4, before=self.tree)  # よく使うので、表の上に置く
        df, dl = self.drag_buttons(gf)
        df.pack(anchor="w")
        dl.pack(anchor="w")
        # 1 点で壁(XYZ に平行な面)を作る。押すと確認して、そのまま保存する
        wf = ttk.LabelFrame(gf, text="壁(1 点): 手先の位置に、x・y・z に平行な壁を作る", padding=4)
        wf.pack(fill="x", pady=(6, 2))
        r1 = ttk.Frame(wf)
        r1.pack(anchor="w")
        ttk.Label(r1, text="この位置より").pack(side="left")
        self.wall_ax = tk.StringVar(value="x")
        for a in "xyz":
            ttk.Radiobutton(r1, text=a, variable=self.wall_ax, value=a).pack(side="left")
        self.wall_side = tk.StringVar(value="+")
        ttk.Radiobutton(r1, text="＋側", variable=self.wall_side, value="+").pack(side="left", padx=(8, 0))
        ttk.Radiobutton(r1, text="−側", variable=self.wall_side, value="-").pack(side="left")
        ttk.Label(r1, text="を禁止").pack(side="left")
        r2 = ttk.Frame(wf)
        r2.pack(anchor="w", pady=2)
        ttk.Label(r2, text="境目を手先から").pack(side="left")
        self.wall_gap = tk.StringVar(value="20")
        ttk.Entry(r2, textvariable=self.wall_gap, width=5).pack(side="left", padx=2)
        ttk.Label(r2, text="mm 先に").pack(side="left")
        tk.Button(r2, text="この位置に壁を作る(保存)", bg="#ffe0e0", font=("Noto Sans CJK JP", 11, "bold"),
                  command=self.zone_wall_here).pack(side="left", padx=8)
        # 2 点以上で箱を作る(物の周りなど)
        bf2 = ttk.LabelFrame(gf, text="箱(2 点以上): 記録した点を囲む箱", padding=4)
        bf2.pack(fill="x", pady=2)
        ttk.Button(bf2, text="① いまの手先を記録", command=self.zone_rec).grid(row=0, column=0, padx=2)
        ttk.Button(bf2, text="記録を消す", command=self.zone_rec_clear).grid(row=0, column=1, padx=2)
        ttk.Button(bf2, text="② 囲む箱を入力欄に入れる", command=self.zone_from_points).grid(row=0, column=2, padx=2)
        self.lbl_rec = tk.Label(bf2, text="記録した点: なし", anchor="w", justify="left", fg="#555")
        self.lbl_rec.grid(row=1, column=0, columnspan=3, sticky="w")
        tk.Label(gf, anchor="w", justify="left", fg="#777", wraplength=530,
                 text="・座標は禁止エリアの判定と同じ(GUI の運動学の手先)。上の TCP(コントローラの値)とは z がずれる。\n"
                      "・境目を 0 mm にすると、手先が境目の上になり、保存したとたんドラッグは止まる(外へ出る動きは受け付ける)。\n"
                      "・箱: 記録した点は RViz に黄色い球、作る予定の箱は黄色い半透明で出る。② のあと「追加 / 更新」で保存。").pack(fill="x")
        self.zpts = []
        bf = ttk.Frame(p)
        bf.pack(anchor="w", pady=4)
        ttk.Button(bf, text="追加 / 更新(同名)", command=self.zone_put).grid(row=0, column=0, padx=2)
        ttk.Button(bf, text="削除", command=self.zone_del).grid(row=0, column=1, padx=2)
        ttk.Button(bf, text="選んだエリアを隠す / 出す", command=self.zone_toggle_hidden).grid(row=0, column=2, padx=(12, 2))
        ttk.Button(bf, text="全部出す", command=self.zone_show_all).grid(row=0, column=3, padx=2)
        self.axis_filter(p).pack(anchor="w")
        tk.Label(p, anchor="w", fg="#777", wraplength=540, justify="left",
                 text="色: x の壁=赤、y の壁=緑、z の壁=青(RViz の軸と同じ)、箱=オレンジ。一覧で選んだエリアは RViz で濃く表示。隠す/出すは表示だけ(判定はそのまま)").pack(fill="x")
        mf = ttk.Frame(p)
        mf.pack(anchor="w")
        ttk.Label(mf, text="腕の余裕 mm").grid(row=0, column=0)
        self.margin = tk.StringVar(value=str(core.LIM.get("link_margin_mm", 80)))
        ttk.Entry(mf, textvariable=self.margin, width=5).grid(row=0, column=1, padx=4)
        ttk.Button(mf, text="反映", command=self.set_margin).grid(row=0, column=2)
        self.lbl_zone = tk.Label(p, text="", anchor="w", justify="left", wraplength=540)
        self.lbl_zone.pack(fill="x", pady=4)

    def build_imp(self, nb):
        p = ttk.Frame(nb, padding=6)
        nb.add(p, text="インピーダンス")
        self.ws = tk.IntVar(value=0)
        ttk.Radiobutton(p, text="関節空間", variable=self.ws, value=0, command=self.fill_imp).pack(anchor="w")
        ttk.Radiobutton(p, text="直交空間(手先)", variable=self.ws, value=1, command=self.fill_imp).pack(anchor="w")
        f = ttk.Frame(p)
        f.pack(anchor="w")
        self.ith = self.entries(f, [f"閾値{i}" for i in range(1, 7)], [10, 10, 10, 1, 1, 1], 0, 6)
        self.im = self.entries(f, [f"m{i}" for i in range(1, 7)], [.04] * 3 + [.01] * 3, 2, 6)
        self.ib = self.entries(f, [f"b{i}" for i in range(1, 7)], [.1] * 3 + [.08] * 3, 4, 6)
        self.ik = self.entries(f, [f"k{i}" for i in range(1, 7)], [0] * 6, 6, 6)
        mf = ttk.Frame(p)
        mf.pack(anchor="w", pady=4)
        self.imx = self.entries(mf, ["最大速度", "最大加速度", "最大角速度", "最大角加速度"], [20, 20, 20, 20], 0, 6)
        # その場で保持: 目標の姿勢のまわりを小さく行き来させ続け、実質の静止状態でインピーダンスを効かせる
        hf = tk.LabelFrame(p, text="今の位置を保持して、インピーダンス制御を確かめる", padx=6, pady=4)
        hf.pack(fill="x", pady=4)
        tk.Label(hf, anchor="w", justify="left", bg="#fff8dc", fg="#333", wraplength=520,
                 text="コントローラのインピーダンスは、軌道(動き)があるときだけ効く。そこで、目標の姿勢のまわりで、全関節を ± 少しだけ\n"
                      "行き来させ続ける。目標は、押したときの今の位置(手順: 開始 → 行き来 → 今の位置へ戻る → 10 秒待つ → 終了)。行き来のあいだに押す").pack(fill="x")
        vf = ttk.Frame(hf)
        vf.pack(anchor="w")
        self.hold_amp, self.hold_s, self.hold_vel = tk.StringVar(value="0.3"), tk.StringVar(value="30"), tk.StringVar(value="2")
        for i, (t, v) in enumerate(((f"行き来の幅 ±deg({core.LIM.get('hold_amp_min_deg', 0.3):g}〜{core.LIM.get('hold_amp_max_deg', 1.0):g})", self.hold_amp),
                                    ("保持の時間 s", self.hold_s), ("速度 %", self.hold_vel))):
            ttk.Label(vf, text=t).grid(row=0, column=2 * i, padx=(0 if i == 0 else 8, 2))
            ttk.Entry(vf, textvariable=v, width=5).grid(row=0, column=2 * i + 1)
        tk.Button(hf, text="▶ 今の位置を保持して、インピーダンスを確かめる", bg="#6a1b9a", fg="white", font=("Noto Sans CJK JP", 12, "bold"),
                  command=self.hold).pack(fill="x", pady=4)
        self.lbl_seq2 = tk.Label(hf, text="", anchor="w", justify="left", fg="#6a1b9a")
        self.lbl_seq2.pack(fill="x")
        bf = ttk.Frame(p)
        bf.pack(anchor="w")
        ttk.Button(bf, text="開始(止まったままでは効かない)", command=lambda: self.imp(1)).grid(row=0, column=0, padx=2)
        ttk.Button(bf, text="停止", command=lambda: self.imp(0)).grid(row=0, column=1, padx=2)
        tk.Label(p, text="(ドラッグモードは「ドラッグ」タブへ移しました)", fg="#777").pack(anchor="w", pady=4)
        ttk.Label(p, text="外力が閾値を超えると腕が譲る(マニュアル 9.33)。軌道(動き)があるときだけ効き、止まったままでは効かない。\nk=0 では、譲ったあと自分では戻らない。事前に衝突レベルの設定が必要(静的衝突検知は OFF)。\nシミュレーションでは設定を保持するだけ。制御中に腕が禁止エリアへ入ったら、自動で停止します。", justify="left", wraplength=540).pack(anchor="w", pady=4)
        self.fill_imp()

    # ------------------------------------------------------------ 操作
    def say(self, r, prefix=""):
        text = (("[ドライラン] " if r.get("dry_run") else "") + (r.get("msg") or ("OK" if r.get("ok") else f"失敗 err={r.get('err')}")))
        self.msg.config(text=prefix + text, fg="#1a7f37" if r.get("ok") else "#c62828")

    def connect(self):
        self.msg.config(text="接続中...", fg="gray")
        self.root.update_idletasks()
        if self.A.be.connected:
            self.A.be.disconnect()
        self.A.impedance_on = False
        self.A.drag_on = False
        self.A.be = self.A.real if self.be.get() == "real" else self.A.sim
        self.safety.set(False)
        self.A.safety_ok = False
        r = self.A.be.connect(self.ip.get())
        core.log(action="connect", backend=self.A.be.name, ip=self.ip.get(), result=r)
        self.say(r)
        if r.get('ok'):
            self.root.after(600, self.use_current)

    def reset_error(self):
        """コントローラの故障をリセットする(腕は動かない)。インピーダンスが入ったままなら切り直す"""
        if self.A.be.sends_to_robot and self.A.be.connected:
            if not messagebox.askokcancel("実機", "コントローラのエラーをリセットします。\n腕は動きません。インピーダンスが入ったままなら切ります。"):
                return
        self.say(self.A.reset_error())

    def set_mode(self, manual):
        """コントローラの手動/自動モードを切り替える(実機では確認してから)"""
        name = "手動" if manual else "自動"
        if self.A.be.sends_to_robot and self.A.be.connected and not self.A.dry():
            if not messagebox.askokcancel("実機", f"コントローラを{name}モードに切り替えます。\n腕は動きません。\n"
                                          "Web 画面でプログラムを実行中なら、先に止めてください。"):
                return
        self.say(self.A.set_mode(manual))

    def ask_quit(self):
        """「終了」: 確認してから、安全に終わらせる(インピーダンス制御を切る → 動作を止める → 接続を閉じる → 終了)"""
        if messagebox.askokcancel("終了", "プログラムを終了します。\nインピーダンス制御を切り、動作を止め、実機との接続を閉じてから終わります。\n腕は今の姿勢で止まります。"):
            core.log(action="quit_button")
            self.shutdown()

    def shutdown(self):
        """終了処理: インピーダンス制御を切る → 動作を止める → 接続を閉じる(bridge 側でも、同じ順で片づける)。2 回目以降は何もしない"""
        if getattr(self, "_down", False):
            return
        self._down = True
        a = self.A
        try:
            if a.be.connected:
                if a.drag_on:
                    core.log(action="shutdown_drag_off", result=a.drag(False))
                if a.impedance_on:
                    core.log(action="shutdown_imp_off", result=a.imp({"on": False, "ws": self.ws.get()}))
                core.log(action="shutdown_stop", result=a.be.stop())
                core.log(action="shutdown_disconnect", result=a.be.disconnect())
            a.real.disconnect()  # 実機の bridge が残っていれば、閉じて終わらせる
        except Exception as e:
            core.log(action="shutdown_error", error=repr(e))
        core.log(action="shutdown")
        try:
            self.root.destroy()
        except tk.TclError:
            pass

    def disconnect(self):
        if self.A.drag_on:
            self.A.drag(False)
        self.A.drag_on = False
        if self.A.impedance_on:
            self.A.imp({"on": False, "ws": 0})
        self.say(self.A.be.disconnect())
        self.A.impedance_on = False

    def stop(self):
        """「■ 停止」: インピーダンス制御が入っていれば先に切り、それから動作を止める"""
        if not self.A.be.connected:
            return self.say({"ok": False, "msg": "未接続"})
        self.A.abort_seq()  # インピーダンスを入れたままの移動の手順も、ここでやめる(このあと移動は送られない)
        ri = None
        if self.A.drag_on:
            rd = self.A.drag(False)
            core.log(action="stop_drag_off", result=rd)
            if self.tau0 is not None:
                core.log(action="drag_torque_max", max_abs_delta_nm=self.tau_max.round(3).tolist())
                self.tau0 = None
            if not rd.get("ok"):
                return self.say({"ok": False, "msg": f"ドラッグモードを抜けられませんでした(err {rd.get('err')})。非常停止を押してください"})
        if self.A.impedance_on:
            ri = self.A.imp({"on": False, "ws": self.ws.get()})
            core.log(action="stop_imp_off", result=ri)
            self.imp_watch(True, {})
        r = self.A.be.stop()
        core.log(action="stop", result=r)
        if ri is not None and not ri.get("ok"):
            return self.say({"ok": False, "msg": f"インピーダンス制御を切れませんでした(err {ri.get('err')})。非常停止を押してください"})
        self.say(r, prefix="インピーダンス制御を切り、" if ri is not None else "")

    def go_home(self):
        """初期姿勢(関節角)へ移動する。速度・範囲・禁止エリアの検査は、ほかの移動と同じ。
        腕が止まり、インピーダンス・ドラッグ・手順が終わっていて、エラーがないときだけ(ボタンもそのときだけ押せる)"""
        ok, why = self.A.safe_idle()
        if not ok:
            return self.say({"ok": False, "msg": f"初期姿勢へ戻れるのは、動作がすべて終わった安全な状態のときだけです({why})"})
        for v, h in zip(self.jnt, core.HOME_DEG):
            v.set(str(h))
        self.move("joint")

    def fl(self, vs):
        return [float(v.get()) for v in vs]

    def nudge(self, i, s):
        try:
            v = self.fl(self.tcp)
            v[i] += s * float(self.step.get())
            self.tcp[i].set(f"{v[i]:.1f}")
        except ValueError:
            pass

    def turn(self, i, s):
        """目標の向き rx/ry/rz(i=0..2)を、1 回分だけ回す。-180〜180 に収める"""
        try:
            v = float(self.tcp[3 + i].get()) + s * float(self.rstep.get())
            self.tcp[3 + i].set(f"{(v + 180.0) % 360.0 - 180.0:.1f}")
        except ValueError:
            pass

    def use_current_joints(self):
        """関節角度の欄に、いまの関節角を入れる(少しだけ動かしたいときの出発点)"""
        if "joints" in self.S:
            for v, x in zip(self.jnt, self.S["joints"]):
                v.set(f"{x:.2f}")
        else:
            self.say({"ok": False, "msg": "関節角が読めていません(未接続)"})

    def use_current(self):
        if "tcp" in self.S:
            for v, x in zip(self.tcp, self.S["tcp"]):
                v.set(f"{x:.1f}")

    def move(self, kind):
        try:
            vel = float(self.vel.get())
            body = {"kind": kind, "vel": vel}
            if kind == "joint":
                body["joints"] = self.fl(self.jnt)
            else:
                body["pose"] = self.fl(self.tcp)
        except ValueError:
            return self.say({"ok": False, "msg": "数値が不正"})
        # 送る前に、実際に送る命令を決める(禁止エリアに入るなら、手前に縮めた目標になる)
        # インピーダンスを入れて移動するときは、手先の目標を関節移動に直したもの(実際に送る命令)を確認に出す
        pm = self.A.plan_move_imp(body) if self.move_imp.get() else self.A.plan_move(body)
        if not pm.get("ok"):
            core.log(action="move_rejected", body=body, msg=pm.get("msg"))
            return self.say(pm)
        cmd_kind, tgt = pm["cmd"]
        if cmd_kind == "movej":
            desc = f"関節 {[round(v, 2) for v in tgt]}"
            if "joints" in self.S:  # どの関節が何度動くかを、確認のダイアログに出す
                d = [b - a for a, b in zip(self.S["joints"], tgt)]
                ch = [f"j{i + 1}: {self.S['joints'][i]:.1f} → {tgt[i]:.1f} ({d[i]:+.1f}°)" for i in range(6) if abs(d[i]) >= 0.05]
                desc += "\n\n動く関節:\n" + ("\n".join(ch) if ch else "なし") + f"\n最大の変化: {max(abs(x) for x in d):.1f}°"
        else:
            desc = f"手先 {[round(v, 1) for v in tgt]}"
            if "tcp" in self.S:
                dp = np.array(tgt[:3]) - np.array(self.S["tcp"][:3])
                desc += f"\n(いまの手先から x {dp[0]:+.0f}, y {dp[1]:+.0f}, z {dp[2]:+.0f} mm、距離 {np.linalg.norm(dp):.0f} mm)"
        if pm.get("note"):
            desc += f"\n\n※ {pm['note']}"
        if self.move_imp.get():
            try:
                ib = self.imp_body(True)
            except ValueError:
                return self.say({"ok": False, "msg": "インピーダンスの値が不正"})
            ic = self.A.imp({**ib, "check_only": True})
            if not ic.get("ok"):
                return self.say(ic)
            pp = ic["params"]
            desc += (f"\n\nインピーダンスを入れて移動({'関節' if ib['ws'] == 0 else '直交'}空間)\n"
                     f"閾値 {pp['threshold']}\nm {pp['m']}\nb {pp['b']}\nk {pp['k']}\n最大 {pp['max_v']}, {pp['max_va']}, {pp['max_w']}, {pp['max_wa']}\n"
                     "開始 → 1 秒後に移動 → 着いて 5 秒後に終了。外力で腕が動きます\n"
                     "ずれたら目標へ送り直し、インピーダンスを切ったあとも、ずれていれば目標へ合わせます")
        if self.A.be.sends_to_robot and not self.A.dry():
            if not messagebox.askokcancel("実機を動かします", f"目標: {desc}\n速度: {vel}%\n\n非常停止に手が届く状態ですか?"):
                return
        if self.move_imp.get():
            return self.say(self.A.move_with_imp(body, ib))
        self.say(self.A.move(body))

    def drag(self, on):
        if on and self.A.be.sends_to_robot and not self.A.dry():
            if not messagebox.askokcancel("実機", "ドラッグモードに入ります。\n手で押すと腕が動く状態になります。"
                                          "\n腕を手で支えられ、非常停止に手が届き、周囲は安全ですか?"):
                return
        was_on = self.A.drag_on
        self.say(self.A.drag(on))
        # インピーダンス制御と比べられるよう、ドラッグ中もトルクの変化を見張る(しきい値はないので、比は出さない)
        if self.A.drag_on and not was_on and tau_key(self.S):
            self.tau0 = np.array(self.S[tau_key(self.S)], float) * TAU_SCALE
            self.tau_max = np.zeros(6)
            self.tau_th = None
        elif was_on and not self.A.drag_on and self.tau0 is not None:
            core.log(action="drag_torque_max", max_abs_delta_nm=self.tau_max.round(3).tolist())
            self.tau0 = None

    def hold(self):
        """今の位置を保持して、インピーダンスを確かめる: 今の姿勢のまわりで小さく行き来しながら、インピーダンスを効かせる"""
        try:
            amp, hs, vel = float(self.hold_amp.get()), float(self.hold_s.get()), float(self.hold_vel.get())
            ib = self.imp_body(True)
        except ValueError:
            return self.say({"ok": False, "msg": "数値が不正"})
        if "joints" in self.S:
            tgt = [round(v, 3) for v in self.S["joints"]]
        else:
            return self.say({"ok": False, "msg": "今の関節角が読めていません(未接続)"})
        ic = self.A.imp({**ib, "check_only": True})
        if not ic.get("ok"):
            return self.say(ic)
        pp = ic["params"]
        desc = f"今の位置を保持: 関節 {[round(v, 2) for v in tgt]}"
        desc += (f"\n全関節を ±{amp:g}° で {hs:g} 秒行き来、速度 {vel:g}%\n戻ってから、インピーダンスを入れたまま 10 秒待って終了\n\n{'関節' if ib['ws'] == 0 else '直交'}空間 閾値 {pp['threshold']}\n"
                 f"m {pp['m']}\nb {pp['b']}\nk {pp['k']}\n最大 {pp['max_v']}, {pp['max_va']}, {pp['max_w']}, {pp['max_wa']}\n外力で腕が動きます")
        if self.A.be.sends_to_robot and not self.A.dry():
            if not messagebox.askokcancel("実機を動かします", desc + "\n\n非常停止に手が届き、周囲は安全ですか?"):
                return
        self.say(self.A.hold_with_imp(tgt, ib, amp_deg=amp, hold_s=hs, vel=vel))

    def imp_body(self, on):
        """「インピーダンス」タブの値(数値でなければ ValueError)"""
        return {"on": bool(on), "ws": self.ws.get(), "threshold": self.fl(self.ith), "m": self.fl(self.im), "b": self.fl(self.ib),
                "k": self.fl(self.ik), **dict(zip(("max_v", "max_va", "max_w", "max_wa"), self.fl(self.imx)))}

    def imp(self, on):
        if on and self.A.seq and self.A.seq.is_alive():
            return self.say({"ok": False, "msg": "インピーダンスを入れたままの移動を、実行中です"})
        try:
            body = self.imp_body(on)
        except ValueError:
            return self.say({"ok": False, "msg": "数値が不正"})
        if on and self.A.be.sends_to_robot and not self.A.dry():
            if not messagebox.askokcancel("実機", "実機でインピーダンス制御を開始します。\n外力で腕が動きます。非常停止に手が届き、周囲は安全ですか?"):
                return
            body["confirm"] = True
        was_on = self.A.impedance_on
        r = self.A.imp(body)
        self.say(r)
        self.imp_watch(was_on, body)

    def imp_watch(self, was_on, body):
        """インピーダンス制御の開始・停止に合わせて、関節トルクの「開始のときからの変化」の見張りを始める・終える"""
        if self.A.impedance_on and not was_on and tau_key(self.S):
            self.tau0 = np.array(self.S[tau_key(self.S)], float) * TAU_SCALE  # 開始のときのトルク(腕の重さなど)を基準にする
            self.tau_max = np.zeros(6)
            self.tau_th = np.array(body["threshold"], float) if body.get("ws", 0) == 0 else None
        elif was_on and not self.A.impedance_on and self.tau0 is not None:
            core.log(action="impedance_torque_max", max_abs_delta_nm=self.tau_max.round(3).tolist())
            self.tau0 = None

    tau0 = None
    tau_max = np.zeros(6)
    tau_th = None

    def fill_imp(self):
        I = core.LIM["impedance"]
        ws = self.ws.get()
        for v, x in zip(self.ith, I["joint_threshold_nm"]["min"] if ws == 0 else I["cart_threshold"]["min"]):
            v.set(str(x))
        D = I["slow_defaults"]  # 既定は上限ではなく、ゆっくりした値(config/limits.json)
        vals = [D["joint_max_v_deg_s"], D["joint_max_va_deg_s2"]] * 2 if ws == 0 else [D["max_v_mm_s"], D["max_va_mm_s2"], D["max_w_deg_s"], D["max_wa_deg_s2"]]
        for v, x in zip(self.imx, vals):
            v.set(str(x))

    # ------------------------------------------------------------ 禁止エリア
    def load_zone_rows(self):
        self.tree.delete(*self.tree.get_children())
        for i, z in enumerate(self.A.zones.items):
            label = z["name"] + (" (手先)" if z.get("target") == "tcp" else "") + (" [隠]" if z["name"] in self.zone_hidden else "")
            tag = f"ax_{zone_axis(z) or 'box'}"
            self.tree.tag_configure(tag, background=zone_row_hex(z))
            self.tree.insert("", "end", iid=z["name"], tags=(tag,), values=("●" if z.get("enabled", True) else "−", label,
                             z["min"][0], z["max"][0], z["min"][1], z["max"][1], z["min"][2], z["max"][2]))

    zone_hidden = set()  # RViz で隠しているエリアの名前(表示だけ。判定には影響しない)

    def zone_toggle_hidden(self):
        sel = self.tree.selection()
        if not sel:
            return self.say({"ok": False, "msg": "一覧からエリアを選んでください"})
        n = sel[0]
        self.zone_hidden ^= {n}
        self.load_zone_rows()
        self.tree.selection_set(n)
        self.say({"ok": True, "msg": f"'{n}' を RViz で{'隠しました' if n in self.zone_hidden else '表示します'}(判定・止める働きはそのまま)"})

    def zone_show_all(self):
        self.zone_hidden = set()
        self.load_zone_rows()
        self.say({"ok": True, "msg": "すべての禁止エリアを RViz に表示します"})

    def zone_select(self, _):
        sel = self.tree.selection()
        if not sel:
            return
        z = next(z for z in self.A.zones.items if z["name"] == sel[0])
        vals = [z["name"], z["min"][0], z["max"][0], z["min"][1], z["max"][1], z["min"][2], z["max"][2]]
        for v, x in zip(self.zv, vals):
            v.set(str(x))
        self.zon.set(z.get("enabled", True))
        self.ztcp.set(z.get("target") == "tcp")

    def zone_put(self):
        try:
            n = self.zv[0].get().strip()
            a = [float(v.get()) for v in self.zv[1:]]
        except ValueError:
            return self.say({"ok": False, "msg": "数値が不正"})
        if not n or a[0] >= a[1] or a[2] >= a[3] or a[4] >= a[5]:
            return self.say({"ok": False, "msg": "名前が空、または min ≧ max"})
        z = {"name": n, "min": [a[0], a[2], a[4]], "max": [a[1], a[3], a[5]], "enabled": self.zon.get()}
        old = next((q for q in self.A.zones.items if q["name"] == n), None)
        if old:  # 画面で編集しない設定(wall・_note など)は、更新しても残す
            z = {**{k: v for k, v in old.items() if k not in z}, **z}
        if self.ztcp.get():
            z["target"] = "tcp"
        else:
            z.pop("target", None)  # 腕全体で判定
        self.A.zones.items = [q for q in self.A.zones.items if q["name"] != n] + [z]
        self.A.zones.save()
        self.load_zone_rows()
        inside = "joints" in self.S and n in self.A.zones.hits(core.arm_points(np.array(self.S["joints"])))
        self.say({"ok": not inside, "msg": f"禁止エリア '{n}' を保存" + ("。いまの腕がこのエリアの中にいます(動作中・ドラッグ中なら止まります。外へ出る動きだけ受け付けます)" if inside else "")})

    def tip_mm(self):
        """いまの手先(禁止エリアの判定に使う点、GUI の運動学)[mm]。関節角が読めなければ None"""
        if "joints" not in self.S:
            return None
        return core.arm_points(np.array(self.S["joints"]))[-1] * 1000

    def zone_rec(self):
        t = self.tip_mm()
        if t is None:
            return self.say({"ok": False, "msg": "関節角が読めていません(未接続)"})
        self.zpts.append(t)
        core.log(action="zone_point_recorded", point_mm=t.round(1).tolist(), n=len(self.zpts))
        self.show_rec()

    def zone_rec_clear(self):
        self.zpts = []
        self.show_rec()

    def show_rec(self):
        if not self.zpts:
            return self.lbl_rec.config(text="記録した点: なし")
        last = self.zpts[-1]
        self.lbl_rec.config(text=f"記録した点: {len(self.zpts)} 点(最後 x={last[0]:.0f} y={last[1]:.0f} z={last[2]:.0f} mm)")

    def set_zone_entries(self, name, lo, hi):
        for v, x in zip(self.zv, [name, lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]]):
            v.set(x if isinstance(x, str) else f"{x:.0f}")
        self.zon.set(True)

    def new_zone_name(self, base):
        names = {z["name"] for z in self.A.zones.items}
        return next(f"{base}{i}" for i in range(1, 1000) if f"{base}{i}" not in names)

    def zone_from_points(self):
        """記録した点をすべて囲む箱を、入力欄に入れる(2 点以上。厚みのない向きは 10 mm にする)"""
        if len(self.zpts) < 2:
            return self.say({"ok": False, "msg": "点を 2 点以上記録してください(箱の向かい合う角など)"})
        a = np.array(self.zpts)
        lo, hi = a.min(0), a.max(0)
        thin = hi - lo < 10
        lo[thin] -= 5
        hi[thin] += 5
        self.set_zone_entries(self.new_zone_name("box"), lo, hi)
        self.ztcp.set(True)
        self.say({"ok": True, "msg": "記録した点を囲む箱を入力欄に入れました。確かめて「追加 / 更新」で保存。"
                                     "いまの手先が箱の中だと、保存したとたん、ドラッグモードは抜けて止まります"})

    def zone_wall_here(self):
        """いまの手先の位置を境に、選んだ軸の ＋側(−側)を禁止する壁を作り、確認して保存する(1 点でよい)"""
        t = self.tip_mm()
        if t is None:
            return self.say({"ok": False, "msg": "関節角が読めていません(未接続)"})
        try:
            gap = float(self.wall_gap.get())
        except ValueError:
            return self.say({"ok": False, "msg": "境目までの距離が数値でない"})
        if not 0 <= gap <= 500:
            return self.say({"ok": False, "msg": "境目までの距離は 0〜500 mm"})
        ax = self.wall_ax.get()
        k, plus, far = "xyz".index(ax), self.wall_side.get() == "+", 2000.0
        lo, hi = np.full(3, -far), np.full(3, far)
        b = t[k] + gap if plus else t[k] - gap
        if plus:
            lo[k] = b
        else:
            hi[k] = b
        name = self.new_zone_name(f"wall_{ax}{'p' if plus else 'm'}")
        rule = f"{ax} {'≧' if plus else '≦'} {b:.0f} mm"
        if not messagebox.askokcancel("壁を作る", f"禁止エリア '{name}' を作って保存します。\n\n{rule} の側を禁止"
                                      f"(いまの手先 {ax} = {t[k]:.0f} mm から {gap:.0f} mm 先)\n手先だけで判定します。"):
            return
        z = {"name": name, "min": [round(float(v), 1) for v in lo], "max": [round(float(v), 1) for v in hi],
             "enabled": True, "target": "tcp", "wall": f"{ax}_{'min' if plus else 'max'}", "wall_mm": 10,
             "_note": f"手先の位置から作った壁: {rule} を禁止"}
        self.A.zones.items = self.A.zones.items + [z]
        self.A.zones.save()
        self.load_zone_rows()
        core.log(action="zone_wall_created", zone=z, tip_mm=t.round(1).tolist())
        inside = name in self.A.zones.hits(core.arm_points(np.array(self.S["joints"])))
        self.say({"ok": not inside, "msg": f"壁 '{name}' を保存({rule} を禁止)" +
                  ("。いまの手先が境目の上なので、ドラッグ中なら止まります(外へ出る動きは受け付けます)" if inside else "")})

    def zone_del(self):
        sel = self.tree.selection()
        if sel:
            self.A.zones.items = [q for q in self.A.zones.items if q["name"] != sel[0]]
            self.A.zones.save()
            self.load_zone_rows()

    def set_margin(self):
        try:
            core.LIM["link_margin_mm"] = float(self.margin.get())
            self.say({"ok": True, "msg": f"余裕 {core.LIM['link_margin_mm']} mm(この起動中のみ。恒久化は config/limits.json の link_margin_mm)"})
        except ValueError:
            self.say({"ok": False, "msg": "数値が不正"})

    # ------------------------------------------------------------ RViz 上のドラッグ操作
    def real_tcp(self):
        """実機に接続中で、手先の位置をコントローラの座標(今のツールの先端)で扱うとき True"""
        return self.A.be.sends_to_robot and self.A.be.connected and "tcp" in self.S and "joints" in self.S

    def disp_offset(self, rpy_deg=None):
        """RViz の腕(GUI の運動学のフランジ)と、コントローラの手先(ツール 1 番の先端)の位置の差 [mm]。
        実機の目標を RViz に描くとき、この分ずらすと、画面の腕の先と目標が合う。シミュレーションでは 0。
        差はツールの長さ分なので、手先の座標系で一定(d_tool)。目標を描くときは、目標の向き rpy_deg で回して使う。
        (今の腕の向きで計算すると、腕が押されて向きが変わるたびに、RViz の目標まで動いてしまうため)"""
        if self.real_tcp():
            p, _ = core.fk(np.radians(self.S["joints"]))
            R_now = core.rot_rpy(*np.radians(self.S["tcp"][3:]))
            self.d_tool = R_now.T @ (p * 1000 - np.array(self.S["tcp"][:3]))
        elif not (self.A.be.sends_to_robot and self.A.be.connected):
            self.d_tool = None
        if getattr(self, "d_tool", None) is None:
            return np.zeros(3)
        if rpy_deg is None:
            rpy_deg = self.fl(self.tcp)[3:]
        return core.rot_rpy(*np.radians(rpy_deg)) @ self.d_tool

    def target_pose(self):
        t = self.fl(self.tcp)
        x, y, z = (np.array(t[:3]) + self.disp_offset(t[3:])) / 1000
        return x, y, z, quat_from_R(core.rot_rpy(*np.radians(t[3:])))

    def make_target(self):
        """RViz で、矢印をドラッグして位置を変える目標マーカー(向きは GUI の「回転」の欄で変える)"""
        im = InteractiveMarker()
        im.header.frame_id, im.name, im.description, im.scale = "base_link", "target", "target", 0.3
        x, y, z, q = self.target_pose()
        im.pose.position.x, im.pose.position.y, im.pose.position.z = x, y, z
        im.pose.orientation.x, im.pose.orientation.y, im.pose.orientation.z, im.pose.orientation.w = q
        vis = InteractiveMarkerControl()
        vis.always_visible = True
        sph = Marker()
        sph.type = Marker.SPHERE
        sph.scale.x = sph.scale.y = sph.scale.z = 0.04
        sph.color.r, sph.color.g, sph.color.b, sph.color.a = 1.0, 0.8, 0.0, 0.95
        vis.markers.append(sph)
        for k, col in enumerate(AXC):  # 目標の姿勢の軸(自分の座標系)
            e = np.eye(3)[k] * 0.12
            a = arrow("", k, "", (0, 0, 0), e, col, 0.008)
            a.header.frame_id = ""
            vis.markers.append(a)
        im.controls.append(vis)
        for ai, (name, (cx, cy, cz)) in enumerate((("x", (1, 0, 0)), ("y", (0, 0, 1)), ("z", (0, -1, 0)))):
            # RViz では位置だけを動かす(回転の輪は出さない)。向きは GUI の「回転」の欄で変える
            for mode, label, orient in ((InteractiveMarkerControl.MOVE_AXIS, "move", InteractiveMarkerControl.FIXED),):
                c = InteractiveMarkerControl()
                c.name, c.interaction_mode, c.orientation_mode = f"{label}_{name}", mode, orient
                c.orientation.w, c.orientation.x, c.orientation.y, c.orientation.z = 1.0, float(cx), float(cy), float(cz)
                n = math.sqrt(2.0)
                c.orientation.w, c.orientation.x, c.orientation.y, c.orientation.z = 1 / n, cx / n, cy / n, cz / n
                if label == "move":  # 常に見える矢印
                    c.always_visible = True  # つかんでドラッグ
                    mk = Marker()
                    mk.type = Marker.ARROW
                    h = math.sqrt(0.5)  # 矢印は +x 向きに作られるので、軸ごとに向きを変える(コントロールの向きは、中の矢印には効かない)
                    mk.pose.orientation.x, mk.pose.orientation.y, mk.pose.orientation.z, mk.pose.orientation.w = ((0.0, 0.0, 0.0, 1.0), (0.0, 0.0, h, h), (0.0, -h, 0.0, h))[ai]
                    for k, v in enumerate(np.eye(3)[ai] * 0.06):
                        setattr(mk.pose.position, "xyz"[k], float(v))
                    mk.scale.x, mk.scale.y, mk.scale.z = 0.16, 0.025, 0.04
                    mk.color.r, mk.color.g, mk.color.b, mk.color.a = (*AXC[ai], 0.9)
                    c.markers.append(mk)
                im.controls.append(c)
        self.ims.insert(im)
        self.ims.setCallback("target", self.on_drag, InteractiveMarkerFeedback.POSE_UPDATE)
        self.ims.applyChanges()
        self.marker_pose = [x, y, z, *q]

    def on_drag(self, fb):
        """ドラッグ中の目標を、入力欄(mm / deg)に反映する。動かすのは「移動」ボタンを押したときだけ"""
        p, o = fb.pose.position, fb.pose.orientation
        pose = core.pose_from(np.array([p.x, p.y, p.z]), R_from_quat(o.x, o.y, o.z, o.w))
        pose[:3] = list(np.array(pose[:3]) - self.disp_offset(pose[3:]))  # 実機は、コントローラの手先の座標に直す(目標の向きで)
        for v, x in zip(self.tcp, pose):
            v.set(f"{x:.1f}")
        self.drag_t = time.time()
        self.marker_pose = [p.x, p.y, p.z, o.x, o.y, o.z, o.w]

    def sync_target(self):
        """入力欄を手で変えたら、RViz のマーカーを追従させる(ドラッグ中は触らない)"""
        if time.time() - self.drag_t < 1.5:
            return
        x, y, z, q = self.target_pose()
        cur = [x, y, z, *q]
        if self.marker_pose is None or max(abs(a - b) for a, b in zip(cur[:3], self.marker_pose[:3])) > 1e-4 or \
                max(abs(a - b) for a, b in zip(cur[3:], self.marker_pose[3:])) > 1e-3:
            from geometry_msgs.msg import Pose
            ps = Pose()
            ps.position.x, ps.position.y, ps.position.z = x, y, z
            ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = q
            self.ims.setPose("target", ps)
            self.ims.applyChanges()
            self.marker_pose = cur

    def spin(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        self.root.after(20, self.spin)

    # ------------------------------------------------------------ 周期処理
    def tick(self):
        """200 ms ごと: 状態を読んで、RViz に関節角を流し、禁止エリアを監視する"""
        try:
            a = self.A
            s = a.be.state() if a.be.connected else {}
            self.S = s
            busy = a.seq is not None and a.seq.is_alive()
            for lb in (self.lbl_seq, self.lbl_seq2):
                lb.config(text=(("▶ " if busy else "") + a.seq_status) if a.seq_status else "")
            # カウントダウン: 手順の実行中だけ、■ 停止 の下に大きく出す
            if busy:
                dl = a.seq_deadline
                txt = a.seq_status + (f"  あと {max(0.0, dl - time.time()):.1f} 秒" if dl else "")
                if not self.lbl_count.winfo_ismapped():
                    self.lbl_count.pack(fill="x", padx=6, pady=(0, 3), after=self.btn_stop)
                self.lbl_count.config(text=txt, bg="#6a1b9a" if a.impedance_on else "#455a64")
            elif self.lbl_count.winfo_ismapped():
                self.lbl_count.pack_forget()
            # 初期姿勢へ戻るボタンは、動作がすべて終わった安全な状態のときだけ押せる
            safe = (a.be.connected and not busy and not a.impedance_on and not a.drag_on and "joints" in s
                    and s.get("done", True) and not s.get("main_err") and not s.get("estop"))
            self.btn_home.config(state="normal" if safe else "disabled", bg="#2e7d32" if safe else "#9e9e9e")
            hj = JointState()  # 初期姿勢(init_pose.csv)を、比較用の半透明の腕に流す
            hj.header.stamp = self.node.get_clock().now().to_msg()
            hj.name = [f"j{i}" for i in range(1, 7)]
            hj.position = [float(np.radians(v)) for v in core.HOME_DEG]
            self.pub_home.publish(hj)
            if not a.be.connected:  # 未接続の間も、RViz に初期姿勢の腕を出しておく
                js = JointState()
                js.header.stamp = self.node.get_clock().now().to_msg()
                js.name = [f"j{i}" for i in range(1, 7)]
                js.position = [float(np.radians(v)) for v in core.HOME_DEG]
                self.pub_js.publish(js)
            if "joints" in s:
                js = JointState()
                js.header.stamp = self.node.get_clock().now().to_msg()
                js.name = [f"j{i}" for i in range(1, 7)]
                js.position = [float(np.radians(v)) for v in s["joints"]]
                self.pub_js.publish(js)
                hits = a.zones.hits(core.arm_points(np.array(s["joints"])))
                moving = (not s.get("done", True)) or a.impedance_on or a.drag_on
                if hits and moving and not self.busy:
                    self.busy = True
                    if a.drag_on:
                        a.drag(False)
                    a.be.stop()
                    if a.impedance_on:
                        a.imp({"on": False, "ws": self.ws.get()})
                    core.log(action="zone_violation_stop", zones=sorted(hits))
                    self.say({"ok": False, "msg": f"動作中に禁止エリアに入ったので停止: {', '.join(sorted(hits))}"})
                if not hits:
                    self.busy = False
                t = s["tcp"]
                extra = ""
                if self.tau0 is None and "jt_tor" in s:
                    extra = f"\nτ[Nm](読み値×{TAU_SCALE:g}、推定) " + " ".join(f"{v * TAU_SCALE:6.1f}" for v in s["jt_tor"])
                if self.tau0 is not None and tau_key(s):  # インピーダンス制御中: 押したことによるトルクの変化
                    d = np.array(s[tau_key(s)], float) * TAU_SCALE - self.tau0
                    self.tau_max = np.maximum(self.tau_max, np.abs(d))
                    extra = f"\nΔτ[Nm](×{TAU_SCALE:g}、推定) " + " ".join(f"{v:6.1f}" for v in d) + "\n最大   " + " ".join(f"{v:6.1f}" for v in self.tau_max)
                    if self.tau_th is not None:
                        extra += "\nしきい値比 " + " ".join(f"{100 * m / t:5.0f}%" for m, t in zip(self.tau_max, self.tau_th))
                self.lbl_pose.config(text="j[deg] " + " ".join(f"{v:7.1f}" for v in s["joints"]) + "\nTCP    " + " ".join(f"{c}={v:.0f}" for c, v in zip(C, t)) + (f"\nエラー main={s['main_err']} sub={s['sub_err']}" if s.get("main_err") else "") + extra)
                self.lbl_zone.config(text=("腕が禁止エリア内: " + ", ".join(sorted(hits))) if hits else "禁止エリアの外", fg="#c62828" if hits else "#1a7f37")
            self.publish_arm()
            # ドラッグの状態と、手先の位置(禁止エリアの判定と同じ座標)を、ドラッグ・禁止エリアのタブに出す
            tip = self.tip_mm()
            dtxt = ("ドラッグ ON(手で動かせる)" if a.drag_on else "ドラッグ OFF") + \
                   ("" if tip is None else f"   手先 x={tip[0]:6.0f} y={tip[1]:6.0f} z={tip[2]:6.0f} mm") + \
                   (f"   記録 {len(self.zpts)} 点" if getattr(self, "zpts", None) else "")
            for lbl in self.drag_labels:
                lbl.config(text=dtxt, fg="#1565c0" if a.drag_on else "#555")
            kind = "sim" if a.be is a.sim else "実機"
            txt = "未接続" if not a.be.connected else f"{kind} 接続中 / {'ドライラン' if a.dry() else ('送信有効' if a.be.sends_to_robot else 'シミュレーション')} / {'動作中' if not s.get('done', True) else '停止中'} / インピーダンス {'ON' if a.impedance_on else 'OFF'} / ドラッグ {'ON' if a.drag_on else 'OFF'}"
            if "enable" in s:  # 実機だけ: コントローラの有効化・モード・ツール番号(読み取り)
                txt += f"\nアーム {'有効' if s['enable'] else '無効'} / {'手動' if s['robot_mode'] == 1 else '自動'}モード / 状態 {ROBOT_STATE.get(s['robot_state'], s['robot_state'])} / ツール座標 {s['tool']}"
            self.lbl_state.config(text=txt, fg="gray" if not a.be.connected else ("#b35900" if a.dry() else "#1a7f37"))
        except Exception as e:  # 画面を落とさない
            self.lbl_state.config(text=f"表示エラー: {e}", fg="#c62828")
        self.root.after(200, self.tick)

    def slow_tick(self):
        """500 ms ごと: 禁止エリアと目標位置のマーカーを RViz に出す"""
        # config/zones.json を見張り、書き換えられたら読み直す(GUI を起動し直さなくても反映される)
        if self.A.zones.reload_if_changed():
            self.load_zone_rows()
            if self.A.zones.error:
                self.say({"ok": False, "msg": self.A.zones.error})
            else:
                self.say({"ok": True, "msg": f"禁止エリアを読み直した(config/zones.json): {', '.join(z['name'] for z in self.A.zones.items) or 'なし'}"})
        try:
            arr = MarkerArray()
            d = Marker()
            d.action = Marker.DELETEALL
            arr.markers.append(d)
            show = self.zone_tab() or self.show_zones.get()  # 「禁止エリア」タブのときは常に表示。ほか(移動・ドラッグ)はチェックで切り替える
            sel = set(self.tree.selection()) if show else set()
            tip = self.tip_mm()
            for i, z in enumerate(self.A.zones.items if show else []):
                if z["name"] in self.zone_hidden or not self.show_axis[zone_axis(z) or "box"].get():
                    continue
                m = Marker()
                m.header.frame_id, m.ns, m.id, m.type = "base_link", "zone", i, Marker.CUBE
                lo, hi = np.array(z["min"]) / 1000, np.array(z["max"]) / 1000
                w = z.get("wall")  # 表示だけを、境目の薄い壁にする(例 "x_min" = x の小さい側の面)。判定は箱全体のまま
                if w:
                    k, t = "xyz".index(w[0]), z.get("wall_mm", 10) / 1000
                    if w.endswith("min"):
                        hi[k] = lo[k] + t
                    else:
                        lo[k] = hi[k] - t
                c = (lo + hi) / 2
                m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, c)
                m.pose.orientation.w = 1.0
                m.scale.x, m.scale.y, m.scale.z = map(float, hi - lo)
                # エリアごとの色。薄く(腕が透けて見えるように)、一覧で選んだものは濃く、無効のものはごく薄く
                alpha = 0.03 if not z.get("enabled", True) else (0.30 if z["name"] in sel else 0.12)
                m.color.r, m.color.g, m.color.b, m.color.a = (*zone_color(z), alpha)
                arr.markers.append(m)
                # 名前: 壁は、腕の先に近いところ(壁の面の上)、箱は真ん中の上に出す
                pos = c.copy()
                if w and tip is not None:
                    pos = np.clip(tip / 1000, lo, hi)
                    pos[k] = c[k]
                pos = pos + np.array([0, 0, 0.05 if not w else 0.0])
                arr.markers.append(text("zone_name", i, pos, z["name"], zone_color(z), 0.045))
            for k, (col, lab) in enumerate(zip(AXC, ("+X", "+Y", "+Z"))):  # ベース座標系の軸(長さ 0.4 m)
                e = np.eye(3)[k] * 0.4
                arr.markers.append(arrow("base_axes", k, "base_link", (0, 0, 0), e, col, 0.015))
                arr.markers.append(text("base_axes_t", k, e * 1.12, lab, col))
            arr.markers.append(text("base_axes_t", 9, (0, 0, -0.06), "BASE", (0.9, 0.9, 0.9), 0.06))
            for i, pt in enumerate(getattr(self, "zpts", [])):  # 記録した点
                m = Marker()
                m.header.frame_id, m.ns, m.id, m.type = "base_link", "zone_points", i, Marker.SPHERE
                m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, np.asarray(pt) / 1000)
                m.pose.orientation.w = 1.0
                m.scale.x = m.scale.y = m.scale.z = 0.03
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.85, 0.0, 0.9
                arr.markers.append(m)
            if len(getattr(self, "zpts", [])) >= 2:  # ② で作る予定の箱(保存する前の見え方)
                pa = np.array(self.zpts) / 1000
                lo, hi = pa.min(0), pa.max(0)
                thin = hi - lo < 0.01
                lo[thin] -= 0.005
                hi[thin] += 0.005
                m = Marker()
                m.header.frame_id, m.ns, m.id, m.type = "base_link", "zone_preview", 0, Marker.CUBE
                m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, (lo + hi) / 2)
                m.pose.orientation.w = 1.0
                m.scale.x, m.scale.y, m.scale.z = map(float, hi - lo)
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.85, 0.0, 0.25
                arr.markers.append(m)
            if "joints" in self.S:  # 現在の手先(TCP)の姿勢の軸
                p, R = core.fk(np.radians(self.S["joints"]))
                for k, col in enumerate(AXC):
                    arr.markers.append(arrow("tcp_axes", k, "base_link", p, p + R[:, k] * 0.1, col, 0.006, 0.6))
            try:
                t = self.fl(self.tcp)
                self.sync_target()
                if self.tcp_last != t:  # 入力が変わったら、目標位置が禁止エリアに入るかを表示
                    self.tcp_last = t
                    q0 = np.array(self.S["joints"]) if "joints" in self.S else np.array(core.HOME_DEG)
                    # 実機は、手先がコントローラの座標(ツール 1 番の先端)なので、コントローラの逆運動学で解く
                    q = self.A.be.ik(t, q0) if self.real_tcp() else core.ik(t, q0)
                    if q is None:
                        self.lbl_preview.config(text="目標: 到達できない姿勢(IK 不可)", fg="#b35900")
                    else:
                        h = self.A.zones.hits(core.arm_points(q))
                        self.lbl_preview.config(text=("目標は禁止エリアの中: " + ", ".join(sorted(h)) + "(移動すると、エリアの手前で止まる)") if h else "目標: 禁止エリアの外(経路は移動時に検査し、入るなら手前で止まる)", fg="#b35900" if h else "#1a7f37")
            except ValueError:
                pass
            for m in arr.markers[1:]:
                m.header.stamp = self.node.get_clock().now().to_msg()
            self.pub_mk.publish(arr)
        except Exception as e:
            self.msg.config(text=f"マーカーエラー: {e}", fg="#c62828")
        self.root.after(500, self.slow_tick)

    tcp_last = None


TAU_SCALE = core.LIM.get("torque_display_scale", 1)  # 実機のトルクの読み値は N·m の約 1/100(config の note を参照)


# 禁止エリアの色: 壁は、その面に垂直な軸の色にそろえる(RViz の軸と同じ。x の壁=赤、y の壁=緑、z の壁=青)。
# 箱(壁でないもの)はオレンジ。GUI の一覧の行は、同じ色を白と混ぜて薄くしたもの
BOX_COLOR = (1.0, 0.6, 0.0)


def zone_axis(z):
    """壁なら、その面に垂直な軸('x'/'y'/'z')。箱なら None"""
    w = z.get("wall")
    return w[0] if w and w[0] in "xyz" else None


def zone_color(z):
    a = zone_axis(z)
    return AXC["xyz".index(a)] if a else BOX_COLOR


def zone_row_hex(z, mix=0.7):
    r, g, b = (int(255 * (c + (1 - c) * mix)) for c in zone_color(z))
    return f"#{r:02x}{g:02x}{b:02x}"


# 腕のリンクとメッシュ(URDF fairino3_v6 の visual。どれも origin は 0)。RViz の RobotModel の代わりに、透明度を切り替えて描く
ARM_LINKS = ["base_link", "shoulder_link", "upperarm_link", "forearm_link", "wrist1_link", "wrist2_link", "wrist3_link"]
ARM_MESH = "package://fairino_description/meshes/fairino3_v6/{}.STL"
ARM_RGB = (0.898, 0.918, 0.929)


def tau_key(s):
    """関節トルクに使う値。実機は jt_tor(各軸の今のトルク N·m)。無ければ torque(駆動器のトルク。止まっていてもほぼ 0 で、実際のトルクではないらしい)"""
    return "jt_tor" if "jt_tor" in s else ("torque" if "torque" in s else None)


ROBOT_STATE = {1: "停止", 2: "運転", 3: "一時停止", 4: "ドラッグ"}  # ROBOT_STATE_PKG.robot_state


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--execute", action="store_true", help="実機に動作を送る(既定はドライラン)")
    a = ap.parse_args()
    root = tk.Tk()
    root.geometry("600x900+1000+0")
    g = Gui(root, a.execute)
    # 終わるとき(窓を閉じる・docker stop の SIGTERM・Ctrl+C)は、実機を安全な状態にしてから終わる
    root.protocol("WM_DELETE_WINDOW", g.shutdown)
    for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
        signal.signal(sig, lambda *_: root.after(0, g.shutdown))
    try:
        root.mainloop()
    finally:
        g.shutdown()


if __name__ == "__main__":
    main()
