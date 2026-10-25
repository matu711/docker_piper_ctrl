#!/usr/bin/env python3
"""Preview the two cameras used by the PiPER dataset recorder."""

import argparse
import sys
import time

import cv2
import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(
        description="Preview front and wrist cameras side by side."
    )
    parser.add_argument("--front", default="/dev/video6")
    parser.add_argument("--wrist", default="/dev/video0")
    parser.add_argument("--single", metavar="DEVICE", help="指定した1台だけを表示する（例: /dev/video1）")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--fourcc", default="MJPG")
    parser.add_argument(
        "--backend",
        choices=("v4l2", "default"),
        default="v4l2",
    )
    return parser.parse_args()


def open_camera(device, args):
    backend = cv2.CAP_V4L2 if args.backend == "v4l2" else cv2.CAP_ANY
    camera = cv2.VideoCapture(device, backend)
    if not camera.isOpened():
        raise RuntimeError("Could not open camera: {}".format(device))

    camera.set(
        cv2.CAP_PROP_FOURCC,
        cv2.VideoWriter_fourcc(*args.fourcc),
    )
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    camera.set(cv2.CAP_PROP_FPS, args.fps)
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    actual_fourcc_value = int(camera.get(cv2.CAP_PROP_FOURCC))
    actual_fourcc = "".join(
        chr((actual_fourcc_value >> (8 * index)) & 0xFF)
        for index in range(4)
    )
    print(
        "{}: {}x{} @ reported {:.2f} FPS, FourCC={}".format(
            device,
            int(camera.get(cv2.CAP_PROP_FRAME_WIDTH)),
            int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            camera.get(cv2.CAP_PROP_FPS),
            actual_fourcc,
        )
    )
    return camera


def read_frame(camera, device, args):
    ok, frame = camera.read()
    if not ok or frame is None:
        raise RuntimeError("Could not read a frame from {}".format(device))
    if frame.shape[1] != args.width or frame.shape[0] != args.height:
        frame = cv2.resize(frame, (args.width, args.height))
    return frame


def label_frame(frame, label):
    cv2.putText(
        frame,
        label,
        (12, 32),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 0, 0),
        4,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        label,
        (12, 32),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )


def preview_single(args):
    """Use the existing capture settings/helpers without opening a second camera."""
    camera = None
    try:
        camera = open_camera(args.single, args)
        print("Press q in the preview window to exit.")
        period = 1.0 / float(args.fps)
        while True:
            started = time.monotonic()
            frame = read_frame(camera, args.single, args)
            label_frame(frame, "CAMERA {}".format(args.single))
            cv2.imshow("PiPER single camera test", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            remaining = period - (time.monotonic() - started)
            if remaining > 0:
                time.sleep(remaining)
    finally:
        if camera is not None:
            camera.release()
        cv2.destroyAllWindows()
    return 0


def main():
    args = parse_args()
    if args.width <= 0 or args.height <= 0 or args.fps <= 0:
        raise ValueError("width, height, and fps must be positive")
    if len(args.fourcc) != 4:
        raise ValueError("fourcc must contain exactly 4 characters")
    if args.single is not None:
        return preview_single(args)

    front = None
    wrist = None
    try:
        front = open_camera(args.front, args)
        wrist = open_camera(args.wrist, args)
        print("Front camera: {}".format(args.front))
        print("Wrist camera: {}".format(args.wrist))
        print("Press q in the preview window to exit.")

        period = 1.0 / float(args.fps)
        while True:
            started = time.monotonic()
            front_frame = read_frame(front, args.front, args)
            wrist_frame = read_frame(wrist, args.wrist, args)
            label_frame(front_frame, "FRONT {}".format(args.front))
            label_frame(wrist_frame, "WRIST {}".format(args.wrist))
            preview = np.concatenate((front_frame, wrist_frame), axis=1)
            cv2.imshow("PiPER dual camera test", preview)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

            remaining = period - (time.monotonic() - started)
            if remaining > 0:
                time.sleep(remaining)
    finally:
        if front is not None:
            front.release()
        if wrist is not None:
            wrist.release()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (RuntimeError, ValueError) as exc:
        print("ERROR: {}".format(exc), file=sys.stderr)
        sys.exit(1)
