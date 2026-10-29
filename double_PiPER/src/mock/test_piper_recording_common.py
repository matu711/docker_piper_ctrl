#!/usr/bin/env python3
"""Synthetic single-arm recording: temporary files only, no ROS nodes or devices."""
import csv
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import rospy
from mcap.reader import make_reader
from sensor_msgs.msg import JointState

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "piper/scripts"))
import piper_recording_common as recorder


class SingleRecordingTests(unittest.TestCase):
    def args(self):
        from types import SimpleNamespace
        return SimpleNamespace(task="synthetic", fps=20, state_dim=7,
            follower_topic="/follower/current_joint_obs", action_topic="/follower/applied_joint_cmd",
            ee_topic="/follower/end_pose", leader_topic="/leader/joint_grip_pose",
            sync_tolerance=.03, sync_wait=.025, camera_sync_tolerance=.03,
            camera_sync_wait=.1, camera_target_tolerance=.1, camera_fourcc="MJPG",
            camera_fps=30, front_camera="front", wrist_camera="wrist", width=640,
            height=480, repo_id="local/test", robot_type="piper_single_arm")

    def test_saved_episode_keeps_state_action_ee_and_images_aligned(self):
        with tempfile.TemporaryDirectory() as root:
            writer = recorder.RawEpisodeWriter(
                Path(root), "synthetic", self.args(),
                ["joint%d" % i for i in range(1, 7)] + ["gripper"],
            )
            state = np.arange(7, dtype=np.float32)
            action = state + 0.25
            ee = np.array([0.1, 0.2, 0.3, 0., 0., 0., 1., 0.04])
            writer.add_frame(
                b"front", b"wrist", state, action, ee,
                10., 10., 10., writer.start_monotonic + 0.05, 0.,
                10., 10., 10., 0., 0., 0.,
            )
            # Capture owns a snapshot even if the caller reuses its buffers.
            state[:] = -1
            action[:] = -1
            output = writer.save()
            self.assertFalse(writer.temp_dir.exists())
            with np.load(output / "data.npz") as data:
                np.testing.assert_array_equal(data["observation_state"], [np.arange(7)])
                np.testing.assert_array_equal(data["action"], [np.arange(7) + 0.25])
                self.assertEqual(data["observation_ee"].shape, (1, 8))
            with (output / "frames.csv").open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 1)
            self.assertEqual((output / rows[0]["front_image"]).read_bytes(), b"front")
            self.assertEqual((output / rows[0]["wrist_image"]).read_bytes(), b"wrist")
            self.assertTrue(json.loads((output / "meta.json").read_text())["successful"])

    def test_empty_episode_cannot_be_saved_and_discard_is_idempotent(self):
        with tempfile.TemporaryDirectory() as root:
            writer = recorder.RawEpisodeWriter(Path(root), "empty", self.args(), [])
            with self.assertRaisesRegex(RuntimeError, "zero-frame"):
                writer.save()
            writer.discard()
            writer.discard()
            self.assertEqual(list(Path(root).iterdir()), [])

    def test_interrupted_mcap_remains_readable_and_not_marked_complete(self):
        with tempfile.TemporaryDirectory() as root:
            writer = recorder.McapEpisodeRecorder()
            writer.start(Path(root))
            state = JointState()
            state.header.stamp = rospy.Time.from_sec(10.)
            state.position = [0.] * 7
            writer.write_ros_message("/follower/current_joint_obs", state)
            output = writer.finish_incomplete()
            self.assertTrue(output.name.endswith(".incomplete.mcap"))
            with output.open("rb") as stream:
                rows = list(make_reader(stream).iter_messages())
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0][1].topic, "/follower/current_joint_obs")
            self.assertEqual(rows[0][2].publish_time, 10_000_000_000)
            self.assertFalse(writer.active)




# Restore fake import modules between suites during unittest discovery.
from _mock_isolation import isolate_imports
setUpModule, tearDownModule = isolate_imports()

if __name__ == "__main__":
    unittest.main()
