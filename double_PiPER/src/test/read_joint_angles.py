#!/usr/bin/env python3
"""現在の関節1〜6の角度を表示する（動作指令なし）。"""
import argparse
import math
import time

from piper_sdk import Piper


def read_updated_state(piper, timeout=3.0):
    """受信時刻が更新された関節情報を返す。"""
    previous_stamp = piper.get_joint_states()[1]
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state = piper.get_joint_states()
        if state[1] > previous_stamp:
            return state
        time.sleep(0.05)
    raise RuntimeError("関節情報が更新されません。電源・CAN接続を確認してください。")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--can", default="can0", help="CANインターフェース（既定: can0）")
    parser.add_argument("--watch", action="store_true", help="受信更新を確認しながら連続表示する")
    parser.add_argument("--rate", type=float, default=5.0, help="連続表示の更新回数/秒（既定: 5）")
    args = parser.parse_args()
    if not math.isfinite(args.rate) or args.rate <= 0:
        parser.error("--rate must be finite and positive")

    piper = Piper(args.can)
    piper.init()
    piper.connect()
    try:
        while True:
            state = read_updated_state(piper)
            for number, angle in enumerate(state[0], start=1):
                print(f"関節{number}: {angle:.6f} rad ({math.degrees(angle):.3f}°)")
            print(f"受信時刻: {state[1]} / 更新周波数: {state[2]} Hz", flush=True)
            if not args.watch:
                break
            time.sleep(1.0 / args.rate)
    except KeyboardInterrupt:
        if not args.watch:
            raise


if __name__ == "__main__":
    main()
