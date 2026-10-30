"""Dual storage schema preserved verbatim from the working reference recorder."""
import csv
import json
import os
import re
import time
from pathlib import Path
from datetime import datetime, timedelta, timezone
import numpy as np
TOKYO = timezone(timedelta(hours=9), name="Asia/Tokyo")
STREAMS = ("right_state", "right_action", "left_state", "left_action")
NAMES = [side+"_"+joint for side in ("right", "left")
         for joint in ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper")]

def reserve_saved_folder(root):
    base = datetime.now(TOKYO).strftime("%Y%m%d_%H%M%S")
    suffix = 0
    while True:
        candidate = root/(base if suffix == 0 else base+"_%02d" % suffix)
        try:
            candidate.mkdir()  # Atomic reservation; never replace an old folder.
            return candidate
        except FileExistsError:
            suffix += 1


def saved_episode_folders(root):
    result = []
    for folder in root.iterdir():
        if not folder.is_dir() or folder.is_symlink() or not re.fullmatch(
                r"(?:\d{8}_\d{6}(?:_\d+)?|episode_\d+)", folder.name):
            continue
        try:
            meta = json.loads((folder/"meta.json").read_text())
            if not isinstance(meta, dict):
                continue
            saved = meta.get("status") == "saved" or (
                meta.get("status") is None and meta.get("successful") is True)
            if saved and meta.get("num_frames", 0) > 0 and all(
                    (folder/name).is_file() for name in ("data.npz", "frames.csv")) and any(
                    f.is_file() and not f.name.endswith(".incomplete.mcap") for f in folder.glob("*.mcap")):
                result.append(folder)
        except (OSError, ValueError, TypeError):
            continue
    return sorted(result)


def atomic_json(path, payload):
    temporary = path.with_suffix(path.suffix+".partial")
    with temporary.open("w") as stream:
        json.dump(payload, stream, indent=2, allow_nan=False)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


class Episode:
    def __init__(self, root, metadata, anchor):
        self.name = "episode_"+str(time.time_ns())
        self.path = root/(".tmp_"+self.name)
        self.path.mkdir()
        self.metadata, self.anchor = dict(metadata), anchor
        self.sides = ("single",) if metadata["state_dim"] == 7 else ("right", "left")
        self.streams = tuple(side+"_"+kind for side in self.sides for kind in ("state", "action"))
        self.names = metadata.get("names", NAMES)
        self.dimension = metadata["state_dim"]
        self.ee_rows = []
        self.rows = []
        self.image_rows = []
        self.camera_names = tuple(self.metadata.get("cameras", []))
        self.last_sequences = dict.fromkeys(self.camera_names, -1)
        self.reused_frames = dict.fromkeys(self.camera_names, 0)
        for name in self.camera_names:
            (self.path/name).mkdir()
        self.dropped = {"missing_or_unsynced": 0, "schedule": 0}
        self.closed = False
        self.csv = (self.path/"frames.partial.csv").open("w", newline="")
        self.writer = csv.writer(self.csv)
        self.writer.writerow(["frame_index", "timestamp", "unix_timestamp"]+
            [key+suffix for key in self.streams for suffix in ("_ros_timestamp", "_time_error")]+
            ["sync_span"]+["state_"+name for name in self.names]+["action_"+name for name in self.names]+
            [name+suffix for name in self.camera_names
             for suffix in ("_ros_timestamp", "_time_error", "_sequence", "_image", "_reused", "_reuse_count")]+
            (["camera_sync_span", "all_sync_span"] if self.camera_names else []) +
            (["ee_x_m", "ee_y_m", "ee_z_m", "ee_qx", "ee_qy", "ee_qz", "ee_qw", "ee_gripper_m", "ee_ros_timestamp", "ee_time_error"] if self.dimension == 7 else []))
        atomic_json(self.path/"recording_status.json", {"status": "recording", "start": anchor})

    def add(self, target, samples, images=None, image_paths=None):
        if self.camera_names and (images is None or image_paths is None):
            raise ValueError("All camera images required for this episode")
        state = np.concatenate([samples[side+"_state"][0] for side in self.sides])
        action = np.concatenate([samples[side+"_action"][0] for side in self.sides])
        stamps = [samples[key][1] for key in self.streams]
        errors = [samples[key][2] for key in self.streams]
        row = (target-self.anchor, target, state, action, stamps, errors, max(stamps)-min(stamps))
        fields = []
        if self.camera_names:
            image_stamps = [images[name][2] for name in self.camera_names]
            image_errors = [stamp-target for stamp in image_stamps]
            sequences = [images[name][0] for name in self.camera_names]
            paths = [image_paths[name] for name in self.camera_names]
            camera_span = max(image_stamps)-min(image_stamps)
            all_span = max(stamps+image_stamps)-min(stamps+image_stamps)
            reused = [images[name][0] == self.last_sequences[name] for name in self.camera_names]
            counts = [self.reused_frames[name]+int(reused[i]) for i, name in enumerate(self.camera_names)]
            image_row = (image_stamps, image_errors, sequences, paths, camera_span, all_span, reused, counts)
            fields = [item for values in zip(image_stamps, image_errors, sequences, paths, reused, counts) for item in values]+[camera_span, all_span]
        if self.dimension == 7:
            ee, stamp, error = samples["ee"]
            observation_ee = np.concatenate((ee, state[6:7]))
            self.ee_rows.append((observation_ee, stamp, error))
            fields += observation_ee.tolist() + [stamp, error]
        self.writer.writerow([len(self.rows), row[0], target]+
            [item for pair in zip(stamps, errors) for item in pair]+[row[6]]+state.tolist()+action.tolist()+fields)
        if self.camera_names:
            self.image_rows.append(image_row)
            self.last_sequences = dict(zip(self.camera_names, sequences))
            self.reused_frames = dict(zip(self.camera_names, counts))
        self.rows.append(row)

    def finish(self, status, reason=""):
        if self.closed:
            return self.path
        count = len(self.rows)
        data = {
            "timestamp": np.asarray([row[0] for row in self.rows], dtype=np.float64),
            "unix_timestamp": np.asarray([row[1] for row in self.rows], dtype=np.float64),
            "observation_state": np.asarray([row[2] for row in self.rows], dtype=np.float32).reshape(count, self.dimension),
            "action": np.asarray([row[3] for row in self.rows], dtype=np.float32).reshape(count, self.dimension),
            "sync_span": np.asarray([row[6] for row in self.rows], dtype=np.float64),
        }
        for index, key in enumerate(self.streams):
            data[key+"_ros_timestamp"] = np.asarray([row[4][index] for row in self.rows], dtype=np.float64)
            data[key+"_time_error"] = np.asarray([row[5][index] for row in self.rows], dtype=np.float64)
        for index, name in enumerate(self.camera_names):
            data[name+"_ros_timestamp"] = np.asarray([row[0][index] for row in self.image_rows], dtype=np.float64)
            data[name+"_time_error"] = np.asarray([row[1][index] for row in self.image_rows], dtype=np.float64)
            data[name+"_sequence"] = np.asarray([row[2][index] for row in self.image_rows], dtype=np.int64)
            data[name+"_image"] = np.asarray([row[3][index] for row in self.image_rows], dtype="<U64")
            data[name+"_reused"] = np.asarray([row[6][index] for row in self.image_rows], dtype=np.bool_)
            data[name+"_reuse_count"] = np.asarray([row[7][index] for row in self.image_rows], dtype=np.int64)
        if self.camera_names:
            data["camera_sync_span"] = np.asarray([row[4] for row in self.image_rows], dtype=np.float64)
            data["all_sync_span"] = np.asarray([row[5] for row in self.image_rows], dtype=np.float64)
        if self.dimension == 7:
            data["observation_ee"] = np.asarray([row[0] for row in self.ee_rows], dtype=np.float32).reshape(count, 8)
            data["ee_ros_timestamp"] = np.asarray([row[1] for row in self.ee_rows], dtype=np.float64)
            data["ee_time_error"] = np.asarray([row[2] for row in self.ee_rows], dtype=np.float32)
        temporary = self.path/"data.npz.partial"
        with temporary.open("wb") as stream:
            np.savez_compressed(stream, **data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(str(temporary), str(self.path/"data.npz"))
        self.csv.flush()
        os.fsync(self.csv.fileno())
        self.csv.close()
        os.replace(str(self.path/"frames.partial.csv"), str(self.path/"frames.csv"))
        reuse_metadata = {"reused_"+name+"_frames": count for name, count in self.reused_frames.items()}
        atomic_json(self.path/"meta.json", dict(self.metadata, **reuse_metadata, status=status, reason=reason,
            num_frames=count, dropped_frames=self.dropped, started_unix_timestamp=self.anchor,
            saved_unix_timestamp=time.time(), successful=status == "saved"))
        (self.path/"recording_status.json").unlink()
        if status == "saved":
            final = reserve_saved_folder(self.path.parent)
            try:
                self.path.rename(final)
            except Exception:
                final.rmdir()  # Only the empty reservation made above.
                raise
            self.path = final
        self.closed = True
        return self.path
