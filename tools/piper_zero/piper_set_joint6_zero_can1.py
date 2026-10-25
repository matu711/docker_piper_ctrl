#!/usr/bin/env python3
"""Persistently set PiPER joint 6's current position as zero on can1.

The motor-zero sequence follows AgileX's official V2 SDK demo:
https://github.com/agilexrobotics/piper_sdk/blob/master/piper_sdk/demo/V2/piper_set_joint_zero.py

This variant fixes the target to can1 / motor 6 and adds explicit safety
confirmations.  It must be run with no other process using can1.
"""

import sys
import time

from piper_sdk import C_PiperInterface_V2


CAN_NAME = "can1"
MOTOR_NUMBER = 6
CONFIRM_START = "can1 joint6"
CONFIRM_WRITE = "SET ZERO"


def ask_exact(prompt, expected):
    answer = input(prompt).strip()
    if answer != expected:
        print("Cancelled: confirmation did not match.")
        return False
    return True


def main():
    print("WARNING: This permanently changes the motor-6 zero point on can1.")
    print("The arm is first enabled and commanded to the existing all-joint zero pose.")
    print("Motor 6 is then disabled, so physically support the wrist/end effector.")
    print("Stop ROS, data recording, and CAN monitoring before continuing.\n")

    if not ask_exact(
        "Type 'can1 joint6' after clearing the workspace and supporting the arm: ",
        CONFIRM_START,
    ):
        return 1

    piper = C_PiperInterface_V2(CAN_NAME)
    piper.ConnectPort()
    time.sleep(0.2)

    try:
        print("Enabling the arm and commanding the existing zero pose...")
        deadline = time.monotonic() + 10.0
        while not piper.EnablePiper():
            if time.monotonic() >= deadline:
                raise RuntimeError("Timed out while enabling the PiPER arm")
            time.sleep(0.01)

        # These calls match the preparation sequence in the official demo.
        piper.ModeCtrl(0x01, 0x01, 30, 0x00)
        piper.JointCtrl(0, 0, 0, 0, 0, 0)

        input(
            "After the arm has stopped at its existing zero pose, "
            "press Enter to disable joint6: "
        )
        piper.DisableArm(MOTOR_NUMBER)
        time.sleep(0.2)

        print("\nJoint6 is disabled.")
        print("Manually align joint6 with the manufacturer's mechanical zero mark.")
        if not ask_exact(
            "Type 'SET ZERO' only after joint6 is correctly aligned: ",
            CONFIRM_WRITE,
        ):
            piper.EnableArm(MOTOR_NUMBER)
            return 1

        # 0xAE is the official persistent motor-zero command.
        piper.JointConfig(MOTOR_NUMBER, 0xAE)
        time.sleep(0.2)
        piper.EnableArm(MOTOR_NUMBER)
        print("Joint6 zero-setting command sent to can1 motor 6.")
        print("Power-cycle the arm, then verify joint6 feedback is approximately 0 deg.")
        return 0
    finally:
        piper.DisconnectPort()


if __name__ == "__main__":
    sys.exit(main())
