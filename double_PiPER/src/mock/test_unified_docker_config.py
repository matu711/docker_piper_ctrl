"""Docker/launch configuration regressions. No Docker daemon or hardware I/O."""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]


class DockerConfigurationTests(unittest.TestCase):
    def test_shell_syntax(self):
        for name in ('build.sh', 'run.sh'):
            subprocess.run(['sh', '-n', str(ROOT/name)], check=True)
        for name in ('piper_shell.sh', 'piper_env.sh', 'entrypoint_setup.sh'):
            subprocess.run(['bash', '-n', str(ROOT/'assets'/name)], check=True)

    def test_build_uses_repository_context_from_another_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            fake = Path(directory)/'docker'
            fake.write_text('#!/bin/sh\nprintf "%s\\n" "$@" > "$PIPER_TEST_ARGS"\n')
            fake.chmod(0o755)
            capture = Path(directory)/'args'
            environment = dict(os.environ, PATH=directory+':'+os.environ['PATH'],
                               PIPER_TEST_ARGS=str(capture), PIPER_IMAGE='unified:test')
            subprocess.run(['sh', str(ROOT/'build.sh'), '--pull=false'], cwd=directory,
                           env=environment, check=True)
            self.assertEqual(capture.read_text().splitlines(),
                             ['build','--pull=false','-t','unified:test',str(ROOT)])

    def test_required_terminal_font_and_japanese_rendering_dependencies(self):
        dockerfile = (ROOT/'Dockerfile').read_text()
        for package in ('terminator xterm python3-pil fonts-noto-cjk', 'locale-gen en_US.UTF-8',
                        'LANG=en_US.UTF-8', '*faceName: Noto Sans Mono CJK JP', '*utf8: 2'):
            self.assertIn(package, dockerfile)
        self.assertIn('COPY assets/piper_shell.sh /piper_shell.sh', dockerfile)
        self.assertIn('ENTRYPOINT ["/entrypoint_setup.sh"]', dockerfile)

    def test_runtime_keeps_cookie_auth_and_expected_workspace(self):
        script = (ROOT/'run.sh').read_text()
        self.assertNotIn('xhost +\n', script)
        self.assertIn('xhost +si:localuser:', script)
        self.assertIn('PIPER_WORKSPACE=/home/${USR_NAME}/docker_piper_ctrl/double_PiPER', script)
        self.assertIn('XMODIFIERS=${XMODIFIERS:-}', script)
        self.assertNotIn('${PIPER_CTRL_DIR}/double_PiPER:/home/${USR_NAME}/double_PiPER', script)

    def test_unified_entrypoints_use_system_python_and_are_executable(self):
        for name in ('control_piper_unified.py','record_piper_mcap_unified.py'):
            path = ROOT/'double_PiPER/src/piper/scripts'/name
            self.assertEqual(path.read_text().splitlines()[0], '#!/usr/bin/python3')
            self.assertTrue(os.access(path, os.X_OK))

    def test_dual_dataset_default_matches_runtime_bind_mount(self):
        path = ROOT/'double_PiPER/src/piper/launch/collect_dual_piper_mcap_unified.launch'
        root = ET.parse(path).getroot()
        self.assertEqual(root.find("arg[@name='dataset_root']").get('default'),
                         '$(env HOME)/piper_dataset/dual')


if __name__ == '__main__':
    unittest.main()
