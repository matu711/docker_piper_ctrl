#!/usr/bin/env python3
"""Camera-only capture check: never imports an arm driver or connects to CAN."""
import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/"piper"/"scripts"))
from mcap2_camera import CameraCapture


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--front", default="/dev/video4")
    parser.add_argument("--wrist1", default="/dev/video2")
    parser.add_argument("--wrist2", default="/dev/video0")
    parser.add_argument("--seconds", type=float, default=10.)
    args = parser.parse_args()
    devices = [(name, getattr(args, name)) for name in ("front", "wrist1", "wrist2")]
    config = dict(width=640, height=480, fps=30, fourcc="MJPG", backend="v4l2", open_timeout=5., read_timeout=2.)
    capture = CameraCapture(devices, config)
    try:
        capture.wait_ready((config['open_timeout']+10)*3+2)
        print(json.dumps(capture.settings), flush=True)
        deadline = time.monotonic()+args.seconds
        while time.monotonic() < deadline:
            snapshots = [view.frame_snapshot() for view in capture.views.values()]
            if any(not items for items in snapshots):
                raise RuntimeError("Missing camera frames")
            time.sleep(.02)
        print("Frames/FPS:", {name: (view.frame_snapshot()[-1][0]+1, round(view.measured_fps(), 2))
                              for name, view in capture.views.items()}, flush=True)
        print('Latest ages/skew:', {name: round((time.time()-view.frame_snapshot()[-1][2])*1000, 2)
                                   for name, view in capture.views.items()}, flush=True)
    finally:
        started = time.monotonic()
        forced = capture.close()
        print("Closed in %.3fs; forced camera shutdown=%s" % (time.monotonic()-started, forced), flush=True)


if __name__ == "__main__":
    main()
