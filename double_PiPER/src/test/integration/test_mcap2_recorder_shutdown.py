#!/usr/bin/env python3
"""Real cameras + synthetic state/action; SIGINT test without ROS/CAN control."""
import json
import signal
import sys
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from mcap.reader import make_reader

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/"piper"/"scripts"))
import record_dual_piper_mcap2 as recorder


class SyntheticVectors:
    def __init__(self, topic, label, size, mcap):
        self.topic, self.label, self.mcap = topic, label, mcap
    def nearest(self, stamp):
        message = JointState()
        message.header.stamp = rospy.Time.from_sec(stamp)
        message.position = [0.]*7
        self.mcap.write_ros_message(self.topic, message)
        return np.zeros(7, dtype=np.float32), stamp, 0.
    def wait_until_at_least(self, target, deadline):
        pass


def main():
    shutdown_callbacks = []
    # Only capture and file writing are real. No ROS node/service or CAN call.
    ros = MagicMock()
    ros.is_shutdown.return_value = False
    ros.get_param.return_value = False
    ros.on_shutdown.side_effect = shutdown_callbacks.append
    ros.Time.now.side_effect = lambda: rospy.Time.from_sec(time.time())
    ros.Time.from_sec.side_effect = rospy.Time.from_sec
    ros.loginfo.side_effect = lambda fmt, *values: print(fmt % values if values else fmt, flush=True)
    ros.logerr_throttle.side_effect = lambda _, message: print('ERROR: '+str(message), flush=True)
    recorder.rospy = ros
    recorder.RosVectorBuffer = SyntheticVectors
    signal.signal(signal.SIGINT, lambda *_: [callback() for callback in shutdown_callbacks])
    with tempfile.TemporaryDirectory(prefix='mcap2_shutdown_') as root:
        args = SimpleNamespace(root=Path(root), repo_id='test/synthetic', task='synthetic shutdown test',
            fps=20, camera_fps=30, width=640, height=480, jpeg_quality=95, backend='v4l2', fourcc='MJPG',
            front_camera='/dev/video4', wrist1_camera='/dev/video2', wrist2_camera='/dev/video0',
            camera_open_timeout=5., camera_read_timeout=2., control_timeout=20., no_preview=True,
            sync_tolerance=.03, sync_wait=.025, sync_delay=.2, camera_sync_tolerance=.03,
            camera_target_tolerance=.1, max_data_gap=1.)
        instance = recorder.Recorder(args)
        instance.control = lambda name, **kw: 'Synthetic control: '+name
        instance.healthy = lambda: None
        try:
            instance.start()
            print('READY_FOR_SIGINT', flush=True)
            instance.run()
        finally:
            instance.shutdown()
        folder = next(Path(root).glob('.tmp_episode_*'))
        with np.load(folder/'data.npz') as data:
            count = data['action'].shape[0]
            assert count > 0 and data['action'].shape[1] == 14
            assert data['observation_state'].shape == data['action'].shape
        with next(folder.glob('*.incomplete.mcap')).open('rb') as stream:
            message_count = len(list(make_reader(stream).iter_messages()))
        for camera in recorder.CAMERAS:
            image = next((folder/camera).glob('*.jpg'))
            assert cv2.imread(str(image)).shape[:2] == (480, 640)
        metadata = json.loads((folder/'meta.json').read_text())
        assert metadata['status'] == 'incomplete'
        assert (folder/'frames.csv').exists()
        print('SHUTDOWN_OK rows=%d mcap_messages=%d dropped=%s' %
              (count, message_count, metadata['dropped_frames']), flush=True)


if __name__ == '__main__':
    main()
