"""Workspace selection and limited generated-output permission repairs."""
import importlib.util
import os
from pathlib import Path
import stat
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('piper_generated_permissions', ROOT/'assets/piper_generated_permissions.py')
permissions = importlib.util.module_from_spec(spec)
spec.loader.exec_module(permissions)


class WorkspaceTests(unittest.TestCase):
    def workspace(self, override=None):
        environment = dict(os.environ, HOME='/home/tester')
        environment.pop('PIPER_WORKSPACE', None)
        if override is not None:
            environment['PIPER_WORKSPACE'] = override
        result = subprocess.run(['bash','-c','source "$1"; _piper_workspace','bash',
                                 str(ROOT/'assets/piper_shell.sh')],
                                 env=environment, check=True, text=True, capture_output=True)
        return result.stdout.strip()

    def test_default_is_repository_workspace(self):
        self.assertEqual(self.workspace(), '/home/tester/docker_piper_ctrl/double_PiPER')

    def test_old_container_environment_migrates_to_repository_workspace(self):
        self.assertEqual(self.workspace('/home/tester/double_PiPER'),
                         '/home/tester/docker_piper_ctrl/double_PiPER')

    def test_explicit_isolated_workspace_is_preserved(self):
        self.assertEqual(self.workspace('/tmp/test-workspace'), '/tmp/test-workspace')


class GeneratedPermissionTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        for name, marker in (('build','CMakeCache.txt'),('devel','.catkin')):
            (self.root/name).mkdir()
            (self.root/name/marker).touch()
        self.source = self.root/'src'
        self.source.mkdir()
        self.source_file = self.source/'protected.py'
        self.source_file.write_text('protected source')
        self.source_file.chmod(0o444)

    def test_only_unwritable_generated_entries_are_repaired_idempotently(self):
        artifact = self.root/'build/Makefile'
        artifact.write_text('generated')
        artifact.chmod(0o444)
        directory = self.root/'devel/generated'
        directory.mkdir()
        directory.chmod(0o555)
        changed = permissions.repair_generated(self.root, os.getuid(), os.getgid())
        self.assertEqual(set(changed), {str(artifact),str(directory)})
        self.assertTrue(artifact.stat().st_mode & stat.S_IWUSR)
        self.assertTrue(directory.stat().st_mode & stat.S_IWUSR)
        self.assertEqual(self.source_file.stat().st_mode & 0o777, 0o444)
        self.assertEqual(permissions.repair_generated(self.root,os.getuid(),os.getgid()), [])

    def test_symlink_in_outputs_does_not_modify_source(self):
        (self.root/'build/linked-source').symlink_to(self.source_file)
        (self.root/'devel/linked-directory').symlink_to(self.source, target_is_directory=True)
        self.assertEqual(permissions.repair_generated(self.root,os.getuid(),os.getgid()), [])
        self.assertEqual(self.source_file.stat().st_mode & 0o777, 0o444)

    def test_unidentified_output_is_rejected_before_any_repair(self):
        (self.root/'devel/.catkin').unlink()
        artifact = self.root/'build/Makefile'; artifact.touch(); artifact.chmod(0o444)
        with self.assertRaisesRegex(ValueError, 'identified catkin'):
            permissions.repair_generated(self.root,os.getuid(),os.getgid())
        self.assertEqual(artifact.stat().st_mode & 0o777, 0o444)

    def test_hardlink_to_source_is_rejected(self):
        os.link(self.source_file, self.root/'build/source-hardlink')
        with self.assertRaisesRegex(ValueError,'multiply-linked'):
            permissions.repair_generated(self.root,os.getuid(),os.getgid())
        self.assertEqual(self.source_file.stat().st_mode & 0o777, 0o444)


if __name__ == '__main__': unittest.main()
