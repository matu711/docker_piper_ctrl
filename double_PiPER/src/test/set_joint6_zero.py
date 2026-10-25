#!/usr/bin/env python3
"""関節6だけを無効化し、現在位置を本体のゼロ点に設定する。"""
import argparse
import math
import time

from piper_sdk import Piper
from read_joint_angles import read_updated_state


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--can", default="can0", help="CANインターフェース（既定: can0）")
    args = parser.parse_args()

    piper = Piper(args.can)
    interface = piper.init()
    piper.connect()
    before = read_updated_state(piper)
    print(f"設定前 J6: {math.degrees(before[0][5]):.3f}°", flush=True)

    interface.DisableArm(motor_num=6, enable_flag=0x01)
    time.sleep(0.5)
    interface.JointConfig(joint_num=6, set_zero=0xAE)

    readings = []
    for _ in range(5):
        time.sleep(0.5)
        state = read_updated_state(piper)
        angle_deg = math.degrees(state[0][5])
        readings.append(angle_deg)
        print(f"設定後 J6: {angle_deg:.3f}° / 受信時刻: {state[1]}")

    print("関節6は再有効化していません。")
    if not all(abs(angle) <= 0.1 for angle in readings):
        raise SystemExit("ゼロ設定の反映を確認できませんでした（判定範囲: ±0.1°）。")
    print("関節6が約0°になったことを確認しました。電源再投入後の保持は別途確認してください。")


if __name__ == "__main__":
    main()
