#!/usr/bin/env python3
"""Read PiPER joint6 feedback from can1 using the official V2 SDK API."""

import math
import time

from piper_sdk import C_PiperInterface_V2


def main():
    piper = C_PiperInterface_V2("can1")
    piper.ConnectPort()
    try:
        time.sleep(0.2)
        raw = piper.GetArmJointMsgs().joint_state.joint_6
        degrees = raw / 1000.0
        radians = math.radians(degrees)
        print("can1 joint6: {:+.3f} deg ({:+.6f} rad), raw={}".format(
            degrees, radians, raw
        ))
    finally:
        piper.DisconnectPort()


if __name__ == "__main__":
    main()
