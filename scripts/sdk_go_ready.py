"""执行 SRDF ready 命名位姿（arm 组）via SDK MoveJ 直控，2026-08-10。

Ready 位姿（config/piper.srdf <group_state name="ready" group="arm">）：
    joint1=0, joint2=1.4919, joint3=0, joint4=0, joint5=-1.22, joint6=0（弧度）

同 sdk_go_zero.py：MoveJ 关节空间（无逆解歧义），低速 5%，
到位判据 = FK 位置稳定（连续 4 次采样变化 <1mm，motion_status 语义不可靠）。
"""
import time
import sys

from piper_sdk import C_PiperInterface_V2

FACTOR_J = 57295.7795  # 弧度 → SDK 单位（1000*180/π）
JOINT_TARGETS_RAD = [0.0, 1.4919, 0.0, 0.0, -1.22, 0.0]


def main():
    piper = C_PiperInterface_V2("can0", dh_is_offset=1)
    piper.ConnectPort()
    t0 = time.time()
    while not piper.EnablePiper():
        time.sleep(0.01)
        if time.time() - t0 > 10:
            print("ERROR: enable timeout", flush=True)
            return 1
    print("enabled, moving to ready pose via MoveJ @5%", flush=True)
    piper.MotionCtrl_2(0x01, 0x01, 5, 0x00)  # MOVE J, 5%
    time.sleep(0.2)
    cmd = [int(round(v * FACTOR_J)) for v in JOINT_TARGETS_RAD]
    print(f"joint targets (SDK units): {cmd}", flush=True)
    # JointCtrl 是流式指令：需循环发送（~20ms），单发一次固件不执行
    # （2026-08-10 实测：单发后 FK 全程不动）。
    piper.EnableFkCal()
    time.sleep(0.5)
    t0 = time.time()
    last_pos = None
    stable_count = 0
    while time.time() - t0 < 120:
        piper.JointCtrl(cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5])
        time.sleep(0.02)
        try:
            fk = piper.GetFK('feedback')[-1]
            pos = (fk[0], fk[1], fk[2])
            if last_pos is not None:
                move = max(abs(a - b) for a, b in zip(pos, last_pos))
                if move < 1.0:
                    stable_count += 1
                else:
                    stable_count = 0
            last_pos = pos
            if int(time.time()) % 2 == 0:
                print(f"FK: xyz=[{pos[0]:.1f},{pos[1]:.1f},{pos[2]:.1f}]mm "
                      f"stable={stable_count}/4", flush=True)
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
