#!/usr/bin/env python3
"""紧急/常规回 zero：MoveJ 关节空间（无逆解歧义，低速 5%），2026-08-08。

用法：python3 sdk_go_zero.py
"""
import time
import sys
import numpy as np

from piper_sdk import C_PiperInterface_V2

FACTOR_J = 57295.7795  # 弧度 → SDK 单位（1000*180/π）


def main():
    piper = C_PiperInterface_V2("can0", dh_is_offset=1)
    piper.ConnectPort()
    t0 = time.time()
    while not piper.EnablePiper():
        time.sleep(0.01)
        if time.time() - t0 > 10:
            print("ERROR: enable timeout"); return 1
    print("enabled, moving to zero via MoveJ @5%", flush=True)
    piper.MotionCtrl_2(0x01, 0x01, 5, 0x00)  # MOVE J, 5%
    time.sleep(0.2)
    piper.JointCtrl(0, 0, 0, 0, 0, 0)
    # 等待到位：FK 位置稳定性判据（motion_status 0x00 语义不可靠，2026-08-08 实测）
    piper.EnableFkCal()
    time.sleep(0.5)
    t0 = time.time()
    last_pos = None
    stable_count = 0
    while time.time() - t0 < 120:
        time.sleep(2.0)
        try:
            fk = piper.GetFK('feedback')[-1]
            pos = np.array(fk[:3])
            if last_pos is not None and np.linalg.norm(pos - last_pos) < 1.0:
                stable_count += 1
            else:
                stable_count = 0
            last_pos = pos
            if stable_count >= 4:
                break
        except Exception:
            pass
    time.sleep(1.0)
    try:
        fk = piper.GetFK('feedback')[-1]
        print(f"final FK joint6: xyz=[{fk[0]:.1f},{fk[1]:.1f},{fk[2]:.1f}]mm "
              f"rpy=[{fk[3]:.2f},{fk[4]:.2f},{fk[5]:.2f}]deg", flush=True)
    except Exception as e:
        print(f"FK unavailable: {e}", flush=True)
    print("done", flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
