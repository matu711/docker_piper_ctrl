#!/usr/bin/env python3
"""Preview three cameras side by side using the dual camera test helpers."""

import argparse
import sys
import time

import cv2
import numpy as np

from test_dual_camera import label_frame, open_camera, read_frame


def parse_args():
    parser = argparse.ArgumentParser(
        description="Preview three cameras side by side. Press q to exit."
    )
    parser.add_argument("--front", default="/dev/video6")
    parser.add_argument("--wrist1", default="/dev/video0")
    parser.add_argument("--wrist2", default="/dev/video2")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--fourcc", default="MJPG")
    parser.add_argument("--backend", choices=("v4l2", "default"), default="v4l2")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.width <= 0 or args.height <= 0 or args.fps <= 0:
        raise ValueError("width, height, and fps must be positive")
    if len(args.fourcc) != 4:
        raise ValueError("fourcc must contain exactly 4 characters")
    devices = (("FRONT", args.front), ("WRIST1", args.wrist1), ("WRIST2", args.wrist2))
    if len({device for _, device in devices}) != 3:
        raise ValueError("Specify three different camera devices")

    cameras = []
    try:
        for label, device in devices:
            camera = open_camera(device, args)
            cameras.append((label, device, camera))
        print("Press q in the preview window to exit.")

        period = 1.0 / float(args.fps)
        while True:
            started = time.monotonic()
            frames = []
            for label, device, camera in cameras:
                frame = read_frame(camera, device, args)
                label_frame(frame, "{} {}".format(label, device))
                frames.append(frame)
            cv2.imshow("PiPER triple camera test", np.concatenate(frames, axis=1))
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            remaining = period - (time.monotonic() - started)
            if remaining > 0:
                time.sleep(remaining)
    finally:
        for _, _, camera in cameras:
            camera.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (RuntimeError, ValueError) as exc:
        print("ERROR: {}".format(exc), file=sys.stderr)
        sys.exit(1)
