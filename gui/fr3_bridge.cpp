// libfairino (C++ SDK) を標準入出力の1行コマンドで包む小さな橋渡し。GUI(Python)から起動される。
// 入力: 1行1コマンド(空白区切り)。出力: 1行1個のJSON {"ok":bool,"err":int,...}
// 動かす系(movej/movel/imp)は非ブロッキングで即座に戻る(動作中も stop が受けられる)。
#include "robot.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

static FRRobot robot;
static bool connected = false;
static bool imp_on = false;  // インピーダンス制御を入れたままか(終了時に切るため)
static bool drag_on = false;  // ドラッグモード(手で動かせる状態)のままか(終了時に抜けるため)

// 終わるとき(quit・標準入力が閉じた・SIGTERM など)に、実機を安全な状態にしてから接続を閉じる:
// インピーダンス制御を切る → 動作を止める → 接続を閉じる
static void safe_shutdown(const char *why) {
    if (!connected) return;
    double z[6] = {0};
    int e0 = drag_on ? robot.DragTeachSwitch(0) : 0;  // 手で動かせる状態のまま残さない
    int e1 = imp_on ? robot.ImpedanceControlStartStop(0, 0, z, z, z, z, 0, 0, 0, 0) : 0;
    int e2 = robot.StopMotion();
    int e3 = robot.CloseRPC();
    connected = imp_on = drag_on = false;
    std::fprintf(stderr, "[bridge] 終了処理(%s): ドラッグ終了=%d インピーダンス停止=%d 動作停止=%d 切断=%d\n", why, e0, e1, e2, e3);
}

static volatile std::sig_atomic_t stop_req = 0;
static void on_signal(int) { stop_req = 1; }  // getline を中断させ、ループの外の終了処理に進ませる

static void reply(int err, const std::string &extra = "") {
    std::printf("{\"ok\":%s,\"err\":%d%s%s}\n", err == 0 ? "true" : "false", err, extra.empty() ? "" : ",", extra.c_str());
    std::fflush(stdout);
}

int main() {
    // SIGTERM・SIGINT・SIGHUP で、読み込みを中断して終了処理に進む(SA_RESTART を付けない)
    struct sigaction sa;
    std::memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_signal;
    for (int sig : {SIGTERM, SIGINT, SIGHUP}) sigaction(sig, &sa, nullptr);
    // SDK 自身のログを残す(接続の失敗の原因を調べるため)。FR3_SDK_LOG に保存先(xxx.log)を渡したときだけ。4=debug
    if (const char *lp = std::getenv("FR3_SDK_LOG")) {
        int e = robot.LoggerInit(0, lp, 5);
        robot.SetLoggerLevel(4);
        std::fprintf(stderr, "[bridge] SDK log: %s (LoggerInit=%d)\n", lp, e);
    }
    std::string line;
    while (!stop_req && std::getline(std::cin, line)) {
        std::istringstream is(line);
        std::string c;
        is >> c;
        std::vector<double> a;
        std::string tok;
        std::string sarg;
        while (is >> tok) {
            // 最後まで数値として読めたときだけ数値にする(stod("192.168.58.2") は 192.168 を返して成功してしまう)
            size_t used = 0;
            try { double v = std::stod(tok, &used); if (used == tok.size()) { a.push_back(v); continue; } } catch (...) {}
            sarg = tok;
        }
        if (c == "quit") { safe_shutdown("quit"); break; }
        if (c == "connect") {
            int e = robot.RPC(sarg.c_str());
            connected = (e == 0);
            char ver[64] = {0};
            if (connected) robot.GetSDKVersion(ver);
            reply(e, std::string("\"sdk\":\"") + ver + "\"");
            continue;
        }
        if (!connected) { reply(-1, "\"msg\":\"not connected\""); continue; }
        if (c == "close") { safe_shutdown("close"); reply(0); }
        else if (c == "state") {
            JointPos j(0, 0, 0, 0, 0, 0);
            DescPose p(0, 0, 0, 0, 0, 0);
            uint8_t done = 0, estop = 0;
            int mc = 0, sc = 0;
            double tq[6] = {0};
            int e1 = robot.GetActualJointPosDegree(1, &j);
            int e2 = robot.GetActualTCPPose(1, &p);
            robot.GetRobotMotionDone(&done);
            robot.GetRobotEmergencyStopState(&estop);
            robot.GetRobotErrorCode(&mc, &sc);
            robot.GetJointDriverTorque(tq);
            // 有効化・モード・動作の状態・ツール番号・フランジ位置(読み取りだけ)
            ROBOT_STATE_PKG pkg;
            std::memset(&pkg, 0, sizeof pkg);
            int e3 = robot.GetRobotRealTimeState(&pkg);
            char b[1400];
            std::snprintf(b, sizeof b,
                "\"joints\":[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],\"tcp\":[%.2f,%.2f,%.2f,%.3f,%.3f,%.3f],"
                "\"done\":%d,\"estop\":%d,\"main_err\":%d,\"sub_err\":%d,\"torque\":[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]",
                j.jPos[0], j.jPos[1], j.jPos[2], j.jPos[3], j.jPos[4], j.jPos[5], p.tran.x, p.tran.y, p.tran.z, p.rpy.rx,
                p.rpy.ry, p.rpy.rz, done, estop, mc, sc, tq[0], tq[1], tq[2], tq[3], tq[4], tq[5]);
            if (e3 == 0) {
                size_t n = std::strlen(b);
                std::snprintf(b + n, sizeof b - n,
                    ",\"enable\":%d,\"robot_mode\":%d,\"robot_state\":%d,\"tool\":%d,\"user\":%d,\"flange\":[%.2f,%.2f,%.2f,%.3f,%.3f,%.3f]"
                    ",\"jt_tor\":[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]",  // 各軸の今のトルク(N·m、ROBOT_STATE_PKG.jt_cur_tor)
                    pkg.rbtEnableState, pkg.robot_mode, pkg.robot_state, pkg.tool, pkg.user, pkg.flange_cur_pos[0],
                    pkg.flange_cur_pos[1], pkg.flange_cur_pos[2], pkg.flange_cur_pos[3], pkg.flange_cur_pos[4], pkg.flange_cur_pos[5],
                    pkg.jt_cur_tor[0], pkg.jt_cur_tor[1], pkg.jt_cur_tor[2], pkg.jt_cur_tor[3], pkg.jt_cur_tor[4], pkg.jt_cur_tor[5]);
            }
            reply(e1 ? e1 : e2, b);
        }
        else if (c == "enable" && a.size() >= 1) reply(robot.RobotEnable((uint8_t)a[0]));
        else if (c == "mode" && a.size() >= 1) reply(robot.Mode((int)a[0]));
        else if (c == "reset") reply(robot.ResetAllError());
        else if (c == "stop") reply(robot.StopMotion());
        else if (c == "movej" && a.size() >= 7) {  // movej j1..j6 vel%
            JointPos j(a[0], a[1], a[2], a[3], a[4], a[5]);
            ExaxisPos ex(0, 0, 0, 0);
            DescPose off(0, 0, 0, 0, 0, 0);
            // SDK は MoveJ の中で、コントローラで今使っているツールで手先の位置を計算して一緒に送る。
            // ツール番号を 0 のまま送ると、関節角と手先の位置が食い違い、エラー 154(関節の目標点のエラー)で断られる。
            // そのため、今のツール番号・工件座標番号を読んで渡す
            int tool = 0, user = 0;
            int et = robot.GetActualTCPNum(1, &tool), eu = robot.GetActualWObjNum(1, &user);
            if (et != 0 || eu != 0) { reply(et ? et : eu, "\"msg\":\"tool/user number read failed\""); continue; }
            char info[64];
            std::snprintf(info, sizeof info, "\"tool\":%d,\"user\":%d", tool, user);
            reply(robot.MoveJ(&j, tool, user, (float)a[6], 100.0f, 100.0f, &ex, 0.0f, 0, &off), info);
        }
        else if (c == "movel" && a.size() >= 7) {  // movel x y z rx ry rz vel%(手先の位置は、今のツール・工件座標で)
            DescPose p(a[0], a[1], a[2], a[3], a[4], a[5]);
            ExaxisPos ex(0, 0, 0, 0);
            DescPose off(0, 0, 0, 0, 0, 0);
            // MoveJ と同じく、今のツール番号・工件座標番号で送る(GetActualTCPPose が返す手先と同じ座標になる)
            int tool = 0, user = 0;
            int et = robot.GetActualTCPNum(1, &tool), eu = robot.GetActualWObjNum(1, &user);
            if (et != 0 || eu != 0) { reply(et ? et : eu, "\"msg\":\"tool/user number read failed\""); continue; }
            char info[64];
            std::snprintf(info, sizeof info, "\"tool\":%d,\"user\":%d", tool, user);
            reply(robot.MoveL(&p, tool, user, (float)a[6], 100.0f, 100.0f, 0.0f, 0, &ex, 0, 0, &off), info);
        }
        else if (c == "ik" && a.size() >= 12) {  // ik x y z rx ry rz ref_j1..j6: コントローラの逆運動学(今のツール、参考の関節角に近い解)
            DescPose p(a[0], a[1], a[2], a[3], a[4], a[5]);
            JointPos ref(a[6], a[7], a[8], a[9], a[10], a[11]), out(0, 0, 0, 0, 0, 0);
            int e = robot.GetInverseKinRef(0, &p, &ref, &out);
            char b[200];
            std::snprintf(b, sizeof b, "\"joints\":[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f]", out.jPos[0], out.jPos[1], out.jPos[2], out.jPos[3], out.jPos[4], out.jPos[5]);
            reply(e, b);
        }
        else if (c == "fk" && a.size() >= 6) {  // fk j1..j6: コントローラの順運動学(今のツールの手先)
            JointPos j(a[0], a[1], a[2], a[3], a[4], a[5]);
            DescPose p(0, 0, 0, 0, 0, 0);
            int e = robot.GetForwardKin(&j, &p);
            char b[200];
            std::snprintf(b, sizeof b, "\"tcp\":[%.3f,%.3f,%.3f,%.4f,%.4f,%.4f]", p.tran.x, p.tran.y, p.tran.z, p.rpy.rx, p.rpy.ry, p.rpy.rz);
            reply(e, b);
        }
        else if (c == "drag" && a.size() >= 1) {  // drag 1/0: ドラッグモード(手で動かせる状態)に入る/抜ける
            int e = robot.DragTeachSwitch((uint8_t)(a[0] != 0));
            if (e == 0) drag_on = (a[0] != 0);
            reply(e);
        }
        else if (c == "imp" && a.size() >= 2) {  // imp status ws th*6 m*6 b*6 k*6 maxV maxVA maxW maxWA
            if (a[0] == 0) {
                double z[6] = {0};
                int e = robot.ImpedanceControlStartStop(0, (int)a[1], z, z, z, z, 0, 0, 0, 0);
                if (e == 0) imp_on = false;
                reply(e);
            } else if (a.size() >= 2 + 24 + 4) {
                double th[6], m[6], b[6], k[6];
                for (int i = 0; i < 6; i++) { th[i] = a[2 + i]; m[i] = a[8 + i]; b[i] = a[14 + i]; k[i] = a[20 + i]; }
                int e = robot.ImpedanceControlStartStop(1, (int)a[1], th, m, b, k, a[26], a[27], a[28], a[29]);
                if (e == 0) imp_on = true;
                reply(e);
            } else reply(-2, "\"msg\":\"bad args\"");
        }
        else reply(-2, "\"msg\":\"unknown command\"");
    }
    safe_shutdown(stop_req ? "シグナル" : (std::cin.eof() ? "入力が閉じた" : "quit/読み込みエラー"));  // GUI が落ちた・強制終了されたときも、ここを通る
    return 0;
}
