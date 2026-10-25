#!/usr/bin/env python3
"""Exercise real SDK factory code with fake ROS/SDK, without importing either."""
import ast
from pathlib import Path
import types
import unittest
from unittest.mock import Mock
import xml.etree.ElementTree as ET


PIPER = Path(__file__).resolve().parents[1] / 'piper'


class SdkJointLimitTests(unittest.TestCase):
    def setUp(self):
        tree = ast.parse((PIPER / 'scripts/piper_ctrl_single_node.py').read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'C_PiperRosNode')
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef)
                      and n.name == '_create_piper_interface')
        self.init = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '__init__')
        self.params = {}
        self.ros = types.SimpleNamespace(
            get_param=lambda name, default: self.params.get(name, default), loginfo=Mock())
        self.sdk = Mock()
        env = dict(rospy=self.ros, C_PiperInterface=self.sdk)
        exec(compile(ast.Module(body=[method], type_ignores=[]), '<SDK factory>', 'exec'), env)
        self.factory = env['_create_piper_interface']

    def test_default_and_explicit_boolean_reach_both_sdk_interfaces_and_logs(self):
        for value in (None, True, False):
            for port in ('can3', 'can1'):
                with self.subTest(value=value, port=port):
                    self.params = {} if value is None else {'~start_sdk_joint_limit': value}
                    expected = True if value is None else value
                    node = types.SimpleNamespace(can_port=port)
                    self.assertIs(self.factory(node), self.sdk.return_value)
                    self.sdk.assert_called_with(can_name=port, start_sdk_joint_limit=expected)
                    self.ros.loginfo.assert_called_with(
                        'PiPER SDK can=%s start_sdk_joint_limit=%s', port, expected)

    def test_invalid_boolean_fails_before_sdk_creation(self):
        for value in ('false', 'true', 0, 1, None):
            self.params = {'~start_sdk_joint_limit': value}
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.factory(types.SimpleNamespace(can_port='can3'))
        self.sdk.assert_not_called()

    def test_factory_is_used_before_existing_connect_and_mode_sequence(self):
        statements = [ast.dump(n) for n in self.init.body]
        sequence = ['self.piper = self._create_piper_interface()',
                    'self.piper.ConnectPort()', 'self.piper.MotionCtrl_2(1, 1, 30, 0)']
        indices = [statements.index(ast.dump(ast.parse(statement).body[0])) for statement in sequence]
        self.assertEqual(indices, sorted(indices))
        self.assertEqual(indices[1], indices[0]+1)
        self.assertEqual(indices[2], indices[1]+1)

    def test_launch_default_and_both_private_boolean_parameters(self):
        launch = ET.parse(PIPER / 'launch/start_vla_dual_piper.launch').getroot()
        arg = launch.findall("arg[@name='start_sdk_joint_limit']")
        self.assertEqual(len(arg), 1)
        self.assertEqual(arg[0].get('default'), 'true')
        groups = launch.findall('group')
        self.assertEqual({g.get('ns') for g in groups}, {'followerL', 'followerR'})
        for group in groups:
            node = group.find("node[@type='piper_ctrl_dual_feedback_node.py']")
            param = node.findall("param[@name='start_sdk_joint_limit']")
            self.assertEqual(len(param), 1)
            self.assertEqual(param[0].get('type'), 'bool')
            self.assertEqual(param[0].get('value'), '$(arg start_sdk_joint_limit)')
        controller = launch.find("node[@name='vla_dual_piper_controller']")
        self.assertIsNone(controller.find("param[@name='start_sdk_joint_limit']"))




# Restore fake import modules between suites during unittest discovery.
from _mock_isolation import isolate_imports
setUpModule, tearDownModule = isolate_imports()

if __name__ == '__main__':
    unittest.main()
