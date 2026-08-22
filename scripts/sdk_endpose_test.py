#!/usr/bin/env python3
"""第 1 步：固件通道最小验证（SDK EndPoseCtrl 低速单点，2026-08-08）。

流程：连接 → 使能 → 打印当前 FK → 低速(5%) MOVE P 到 demo 初始位姿 → 等到达 → 打印 FK 回验。
安全：move_spd_rate_ctrl=5（百分比）；目标 = 官方 demo 初始位姿（可达、距 zero 小幅）。
前提：ROS 驱动已停（双源隔离）；机械臂当前应在 zero。
"""
import time
import sys
import numpy as np

from piper_sdk import C_PiperInterface_V2

FACTOR = 1000  # 0.001mm / 0.001°

import json

# 目标可从命令行传入：python3 sdk_endpose_test.py '[57,0,600,0,85,0]'（mm/度）
TARGET = [57.0, 0.0, 240.0, 0.0, 85.0, 0.0]  # 默认：Z+25mm 小幅动作（mm/度，XYZ 欧拉角）


def fmt_pose(p):
    return f"xyz=[{p[0]:.1f},{p[1]:.1f},{p[2]:.1f}]mm rpy=[{p[3]:.2f},{p[4]:.2f},{p[5]:.2f}]deg"


def main():
    global TARGET
    if len(sys.argv) > 1:
        TARGET = json.loads(sys.argv[1])
    piper = C_PiperInterface_V2("can0", dh_is_offset=1)
    piper.ConnectPort()
    print("connected, enabling...", flush=True)
    t0 = time.time()
    while not piper.EnablePiper():
        time.sleep(0.01)
        if time.time() - t0 > 10:
            print("ERROR: enable timeout"); return 1
    print("enabled OK", flush=True)
    time.sleep(0.5)

    # 当前状态（SDK 反馈 FK + 固件末端消息）
    piper.EnableFkCal()
    time.sleep(0.5)
    try:
        fk = piper.GetFK('feedback')[-1]
        print(f"GetFK(feedback) joint6: {fmt_pose(fk)}", flush=True)
    except Exception as e:
        print(f"GetFK(feedback) unavailable: {e}", flush=True)
    try:
        print("GetArmEndPoseMsgs:", piper.GetArmEndPoseMsgs(), flush=True)
    except Exception as e:
        print("GetArmEndPoseMsgs error:", e, flush=True)

    # 低速 MOVE P 到目标
    print(f"EndPoseCtrl -> {TARGET} (mm/deg), speed=5%", flush=True)
    piper.MotionCtrl_2(0x01, 0x00, 5, 0x00)  # CAN 控制, MOVE P, 5% 速度
    time.sleep(0.2)
    X, Y, Z, RX, RY, RZ = [round(v * FACTOR) for v in TARGET]
    piper.EndPoseCtrl(X, Y, Z, RX, RY, RZ)

    # 等待运动结束：FK 位置稳定性判据（连续 5 次采样变化 <1mm）
    # （motion_status 0x00 语义不可靠：运动开始时即报告 0x00，2026-08-08 实测）
    t0 = time.time()
    last_pos = None
    stable_count = 0
    while time.time() - t0 < 120:
        time.sleep(1.0)
        try:
            fk = piper.GetFK('feedback')[-1]
            pos = np.array(fk[:3])
            if last_pos is not None and np.linalg.norm(pos - last_pos) < 1.0:
                stable_count += 1
            else:
                stable_count = 0
            last_pos = pos
            if stable_count >= 5:
                print(f"motion settled after {time.time()-t0:.1f}s "
                      f"@ xyz=[{pos[0]:.1f},{pos[1]:.1f},{pos[2]:.1f}]", flush=True)
                break
        except Exception:
            pass
    time.sleep(1.0)

    # 回验（SDK FK）
    try:
        fk = piper.GetFK('feedback')[-1]
        print(f"final FK joint6: {fmt_pose(fk)}", flush=True)
        dev = [abs(fk[i] - TARGET[i]) for i in range(6)]
        print(f"deviation vs target: pos={np.linalg.norm(dev[:3]):.2f}mm "
              f"rpy={max(dev[3:]):.2f}deg", flush=True)
    except Exception as e:
        print(f"FK feedback unavailable: {e}", flush=True)
    # 回验（固件 GetArmEndPoseMsgs）
    try:
        msg = piper.GetArmEndPoseMsgs()
        ep = msg.ArmMsgFeedBackEndPose
        print(f"GetArmEndPoseMsgs: X={ep.X_axis/1000:.1f} Y={ep.Y_axis/1000:.1f} "
              f"Z={ep.Z_axis/1000:.1f} RX={ep.RX_axis/1000:.1f} RY={ep.RY_axis/1000:.1f} "
              f"RZ={ep.RZ_axis/1000:.1f}", flush=True)
    except Exception as e:
        print(f"GetArmEndPoseMsgs unavailable: {e}", flush=True)
    print("done", flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
