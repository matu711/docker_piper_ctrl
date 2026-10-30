"""Shared display additions; input/key conditions live in the reference recorder."""
import textwrap
import cv2
import numpy as np


def detail_lines(recorder):
    episode = recorder.episode
    task = episode.metadata["task"] if episode else recorder.current_task
    rows = len(episode.rows) if episode else 0
    lines = ["Episode: %s | Frames: %d | Record FPS: %g" %
             (episode.name if episode else "-", rows, recorder.fps)]
    if recorder.camera_capture:
        lines.append("Camera FPS: " + "  ".join("%s=%.1f" %
            (name, recorder.camera_capture.views[name].measured_fps()) for name in recorder.cameras))
    lines.extend(textwrap.wrap("Task: " + task, 110) or ["Task:"])
    if recorder.camera_error:
        lines.extend(textwrap.wrap("ERROR: " + recorder.camera_error, 110))
    lines.extend(textwrap.wrap("Result: " + recorder.last_result, 110))
    # No inferred following state: the reference exposes no authoritative status.
    lines.append("Robot control: independent | t=task r=start s=save x=discard q=quit")
    return lines


def draw_details(image, recorder):
    lines = detail_lines(recorder)
    footer = np.zeros((28 * len(lines) + 12, image.shape[1], 3), dtype=np.uint8)
    for index, line in enumerate(lines):
        # OpenCV Hershey fonts do not contain Japanese glyphs. Keep the exact
        # Unicode task in the terminal; render escaped text rather than ????
        # when optional Pillow/CJK fonts are absent.
        line = line.encode("ascii", "backslashreplace").decode("ascii")
        cv2.putText(footer, line, (10, 25 + 28 * index), cv2.FONT_HERSHEY_SIMPLEX,
                    .55, (255, 255, 255), 1)
    if any(not line.isascii() for line in lines):
        try:
            from PIL import Image, ImageDraw, ImageFont
            from pathlib import Path
            font_path = next(path for path in (
                Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"),
                Path("/usr/share/fonts/truetype/fonts-japanese-gothic.ttf")) if path.exists())
            canvas = Image.fromarray(np.zeros_like(footer))
            draw = ImageDraw.Draw(canvas)
            font = ImageFont.truetype(str(font_path), 18)
            for index, line in enumerate(lines):
                draw.text((10, 5 + 28*index), line, font=font, fill=(255,255,255))
            footer = np.array(canvas)
        except (ImportError, OSError, StopIteration):
            pass
    return np.concatenate((image, footer), axis=0)
