#!/usr/bin/env python3
"""グリッパーを無効化し、現在位置を本体のゼロ点に設定する。"""
import argparse
import math
import time

from piper_sdk import Piper


ZERO_TOLERANCE_M = 0.0001  # ±0.1 mm


def read_updated_gripper_state(piper, timeout=3.0):
    """受信時刻が更新された、有効なグリッパー情報を返す。"""
    previous_stamp = piper.get_gripper_states()[1]
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state = piper.get_gripper_states()
        if math.isfinite(state[1]) and state[1] > max(previous_stamp, 0):
            if len(state[0]) != 2 or not all(math.isfinite(value) for value in state[0]):
                raise RuntimeError("グリッパー情報が不正なため中止します。")
            return state
        time.sleep(0.05)
    raise RuntimeError("グリッパー情報が更新されません。電源・CAN接続を確認してください。")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--can", default="can0", help="CANインターフェース（既定: can0）")
    args = parser.parse_args()

    piper = Piper(args.can)
    interface = piper.init()
    piper.connect()
    before = read_updated_gripper_state(piper)
    print(f"対象CAN: {args.can}", flush=True)
    print(f"設定前 グリッパー: {before[0][0] * 1000:.3f} mm", flush=True)

    # 0x00で無効化。SDKのdisable_gripper()が使う0x02（エラー解除）は使わない。
    # 関節モーターには指令を送らない。
    interface.GripperCtrl(gripper_angle=0, gripper_effort=0,
                          gripper_code=0x00, set_zero=0x00)
    time.sleep(0.5)
    interface.GripperCtrl(gripper_angle=0, gripper_effort=0,
                          gripper_code=0x00, set_zero=0xAE)

    readings = []
    try:
        for _ in range(5):
            time.sleep(0.5)
            state = read_updated_gripper_state(piper)
            position = state[0][0]
            readings.append(position)
            print(f"設定後 グリッパー: {position * 1000:.3f} mm"
                  f" / 受信時刻: {state[1]} / {state[2]} Hz")
    finally:
        print("グリッパーは再有効化していません。")

    if not all(abs(position) <= ZERO_TOLERANCE_M for position in readings):
        raise SystemExit("ゼロ設定の反映を確認できませんでした（判定範囲: ±0.1 mm）。")
    print("グリッパーが約0 mmになったことを確認しました。")
    print("電源再投入後の設定保持は別途確認してください。")


if __name__ == "__main__":
    try:
        main()
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from exc
