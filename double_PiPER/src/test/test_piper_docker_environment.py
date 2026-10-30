#!/usr/bin/python3
"""Run inside the PiPER image, with no CAN/camera devices and --network none.

Builds a temporary catkin workspace, imports modules without driver instances,
loads launch XML without starting ROS, and optionally opens a short-lived xterm.
"""
import importlib
import json
import locale
import os
from pathlib import Path
import runpy
import shutil
import subprocess
import sys
import tempfile
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT/'double_PiPER/src/piper/scripts'
sys.path.insert(0, str(SCRIPTS))


class DockerEnvironmentTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(prefix='piper-env-')
        cls.addClassCleanup(cls.temporary.cleanup)
        cls.workspace = Path(cls.temporary.name)/'workspace'
        source = cls.workspace/'src'
        source.mkdir(parents=True)
        for name in ('piper','piper_msgs','piper_vla_runtime_msgs'):
            shutil.copytree(ROOT/'double_PiPER/src'/name, source/name,
                            ignore=shutil.ignore_patterns('__pycache__'))
        # Reproduce relocated catkin output and the missing-pyenv shebang.
        (cls.workspace/'build').mkdir()
        (cls.workspace/'build/CMakeCache.txt').write_text(
            'CMAKE_HOME_DIRECTORY:INTERNAL=/missing/old/workspace/src\n'
            'CMAKE_CACHEFILE_DIR:INTERNAL=/missing/old/workspace/build\n'
            'PYTHON_EXECUTABLE:FILEPATH=/missing/.pyenv/shims/python3\n')
        (cls.workspace/'devel').mkdir()
        (cls.workspace/'devel/.catkin').write_text('/missing/old/workspace/src')
        (cls.workspace/'devel/_setup_util.py').write_text('#!/missing/.pyenv/shims/python3\n')
        cls.build_log = cls.workspace.parent/'build.log'
        environment = dict(os.environ, PIPER_WORKSPACE=str(cls.workspace))
        with cls.build_log.open('w') as log:
            result = subprocess.run(['bash','-c',
                'source /piper_shell.sh && _piper_build_workspace -j2'],
                env=environment, stdout=log, stderr=subprocess.STDOUT, timeout=120)
        if result.returncode:
            raise AssertionError(cls.build_log.read_text())
        cls.generated = cls.workspace/'devel/lib/python3/dist-packages'
        sys.path.insert(0, str(cls.generated))

    def test_relocated_and_pyenv_outputs_are_backed_up_and_regenerated(self):
        backups = list(self.workspace.glob('.piper-generated-backup.*'))
        self.assertEqual(len(backups), 1)
        self.assertIn('/missing/.pyenv/', (backups[0]/'devel/_setup_util.py').read_text())
        self.assertEqual((self.workspace/'devel/_setup_util.py').read_text().splitlines()[0],
                         '#!/usr/bin/python3')
        self.assertIn('PYTHON_EXECUTABLE:FILEPATH=/usr/bin/python3',
                      (self.workspace/'build/CMakeCache.txt').read_text())

    def test_same_workspace_stale_python_cache_is_repaired_without_removal(self):
        cache = self.workspace/'build/CMakeCache.txt'
        content = cache.read_text().replace('PYTHON_EXECUTABLE:FILEPATH=/usr/bin/python3',
                     'PYTHON_EXECUTABLE:FILEPATH=/missing/.pyenv/shims/python3')
        cache.write_text(content)
        (self.workspace/'devel/_setup_util.py').write_text('#!/missing/.pyenv/shims/python3\n')
        environment = dict(os.environ, PIPER_WORKSPACE=str(self.workspace))
        result = subprocess.run(['bash','-c','source /piper_shell.sh && _piper_build_workspace -j2'],
            env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=120)
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertEqual(len(list(self.workspace.glob('.piper-generated-backup.*'))), 1)
        self.assertEqual((self.workspace/'devel/_setup_util.py').read_text().splitlines()[0], '#!/usr/bin/python3')

    def test_system_python_imports_and_all_collection_files_without_hardware(self):
        self.assertEqual(sys.executable, '/usr/bin/python3')
        self.assertTrue(shutil.which('xterm'))
        self.assertIn('utf', locale.getpreferredencoding(False).lower())
        import numpy
        import rospy
        import cv2
        import yaml
        import PIL
        import mcap
        import mcap_ros1
        import piper_sdk
        from piper_msgs.msg import PiperEulerPose
        from piper_vla_runtime_msgs.srv import StartInference
        with patch.object(piper_sdk, 'Piper', side_effect=AssertionError('No SDK instance allowed')), \
             patch.object(cv2, 'VideoCapture', side_effect=AssertionError('No camera allowed')):
            for name in ('control_piper','record_piper_mcap','piper_recording_common','piper_recorder',
                         'piper_config','piper_storage','piper_single',
                         'piper_gui','piper_follower',
                         'piper_ctrl_leader_node_new','piper_ctrl_follower_node',
                         'vla_piper_controller','vla_dual_piper_controller',
                         'piper_ctrl_dual_feedback_node','vla_dual_camera_publisher',
                         'go_initial_pose','go_dual_initial_pose'):
                importlib.import_module(name)

    def test_ros_launch_loads_and_resolves_latest_source_without_starting_nodes(self):
        environment = dict(os.environ, PIPER_WORKSPACE=str(self.workspace))
        command = '''source /piper_shell.sh && sorc_devel && /usr/bin/python3 - <<'PROBE'
import os
from pathlib import Path
import roslaunch
from roslib.packages import find_node
root = Path(os.environ['PIPER_WORKSPACE'])/'src/piper'
for path in root.joinpath('launch').glob('*.launch'):
    config = roslaunch.config.ROSLaunchConfig()
    roslaunch.xmlloader.XmlLoader().load(str(path), config, verbose=False)
    for node in config.nodes:
        resolved = find_node(node.package, node.type)
        assert resolved, (node.package, node.type)
        Path(resolved[0]).resolve().relative_to(root.resolve())
print('latest workspace launch XML and scripts: OK')
PROBE'''
        result = subprocess.run(['bash','-c', command], env=environment, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout)

    def test_japanese_task_renders_with_pillow_and_cjk_font(self):
        import numpy as np
        from PIL import ImageFont
        import piper_gui as gui
        font = ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',18)
        self.assertIsNotNone(font.getmask('袋を持つ').getbbox())
        recorder = SimpleNamespace(episode=None, current_task='袋を持つ', fps=20,
            camera_capture=None, camera_error=None, last_result='Ready')
        image = np.zeros((270,960,3), dtype=np.uint8)
        rendered = gui.draw_details(image, recorder)
        with patch.dict(sys.modules, {'PIL': None}):
            escaped = gui.draw_details(image, recorder)
        self.assertEqual(rendered.shape, escaped.shape)
        self.assertFalse(np.array_equal(rendered, escaped))

    @unittest.skipUnless(os.environ.get('DISPLAY'), 'No authenticated X11 display provided')
    def test_opencv_preview_can_connect_to_authenticated_x11(self):
        import cv2
        import numpy as np
        try:
            cv2.imshow('PiPER MCAP offline preview test', np.zeros((270,960,3), dtype=np.uint8))
            cv2.waitKey(100)
        finally:
            cv2.destroyAllWindows()

    @unittest.skipUnless(os.environ.get('DISPLAY'), 'No authenticated X11 display provided')
    def test_authenticated_xterm_runs_recorder_entrypoint_with_tty(self):
        marker = self.workspace.parent/'xterm-result.json'
        environment = dict(os.environ, PYTHONPATH=str(self.generated)+os.pathsep+os.environ.get('PYTHONPATH',''))
        result = subprocess.run(['xterm','-T','PiPER MCAP offline test','-e',sys.executable,
                                 str(Path(__file__).resolve()),'--xterm-child',str(marker)],
                                env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout)
        payload = json.loads(marker.read_text())
        self.assertTrue(payload['tty'])
        self.assertIn('utf', payload['encoding'].lower())
        self.assertEqual(payload['task'], '袋を持つ')


def xterm_child(marker):
    # Execute the actual thin entrypoint, replacing only its main callback.
    # No ROS node, camera worker, CAN socket or SDK instance is created.
    import piper_recorder as recorder
    def probe():
        Path(marker).write_text(json.dumps(dict(tty=os.isatty(0),
            encoding=sys.stdin.encoding, task='袋を持つ'), ensure_ascii=False))
        print('PiPER UTF-8: 袋を持つ — offline test complete', flush=True)
        time.sleep(.5)
    with patch.object(recorder, 'main', probe):
        runpy.run_path(str(SCRIPTS/'record_piper_mcap.py'), run_name='__main__')


if __name__ == '__main__':
    if len(sys.argv) == 3 and sys.argv[1] == '--xterm-child':
        xterm_child(sys.argv[2])
    else:
        unittest.main()
