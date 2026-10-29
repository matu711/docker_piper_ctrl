"""ROS XmlLoader only: never start a ROS master, driver or Controller.

Numeric fixtures below are synthetic and are not commissioned robot limits.
"""
from pathlib import Path
import ast
import tempfile
import types
import unittest
from unittest.mock import Mock

try:
    import roslaunch
except ImportError:
    roslaunch = None

ROOT = Path(__file__).resolve().parents[1] / "piper"
LAUNCH = ROOT / "launch/start_vla_dual_piper.launch"


@unittest.skipIf(roslaunch is None, "ROS XmlLoader unavailable")
class LaunchParameterExpansionTests(unittest.TestCase):
    def load(self, arguments):
        config = roslaunch.config.ROSLaunchConfig()
        roslaunch.xmlloader.XmlLoader().load(str(LAUNCH), config, argv=arguments,
                                           verbose=False)
        return config

    def test_default_twelve_step_limits_are_point_one(self):
        config = self.load([])
        for side in ("left", "right"):
            self.assertEqual(config.params["/vla_dual_piper_controller/max_delta_" + side].value, [.1]*6)

    def test_explicit_angle_switches_reach_both_driver_factories_and_controller(self):
        tree = ast.parse((ROOT / 'scripts/piper_ctrl_dual_feedback_node.py').read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'DualFeedbackNode')
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_create_piper_interface')
        for enabled in (True, False):
            args = [] if enabled else ['start_sdk_joint_limit:=false', 'controller_joint_limit_enabled:=false',
                                      'dry_run:=true', 'auto_enable:=false']
            config = self.load(args)
            self.assertIs(config.params['/vla_dual_piper_controller/controller_joint_limit_enabled'].value, enabled)
            for side, ns in [('left','followerL'), ('right','followerR')]:
                sdk_flag = config.params['/' + ns + '/piper_ctrl_single_node/start_sdk_joint_limit'].value
                self.assertIs(sdk_flag, enabled)
                factory = Mock()
                env = dict(rospy=types.SimpleNamespace(get_param=lambda name, default: sdk_flag),
                           TrackedPiperInterface=factory)
                exec(compile(ast.Module(body=[method], type_ignores=[]), '<dual SDK factory>', 'exec'), env)
                port = config.params['/' + ns + '/piper_ctrl_single_node/can_port'].value
                env['_create_piper_interface'](types.SimpleNamespace(can_port=port))
                factory.assert_called_once_with(can_name=port, start_sdk_joint_limit=enabled)
            for name, expected in [('z_enabled',True), ('z_min',.01), ('state_timeout_s',.5),
                                   ('gripper_open_width_m',.1), ('gripper_saturation_margin_m',.01)]:
                self.assertEqual(config.params['/vla_dual_piper_controller/'+name].value, expected)

    def test_file_only_and_override_precedence_and_missing_keys(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "limits.yaml"
            path.write_text("max_delta_left: [1,2,3,4,5,6]\nmax_delta_right: [6,5,4,3,2,1]\n")
            config = self.load(["motion_limits_file:=" + str(path)])
            self.assertEqual(config.params["/vla_dual_piper_controller/max_delta_left"].value, [1,2,3,4,5,6])
            self.assertEqual(config.params["/vla_dual_piper_controller/max_delta_right"].value, [6,5,4,3,2,1])
            config = self.load(["motion_limits_file:=" + str(path), "max_delta_left:=[6,5,4,3,2,1]"])
            self.assertEqual(config.params["/vla_dual_piper_controller/max_delta_left"].value, [6,5,4,3,2,1])
            self.assertEqual(config.params["/vla_dual_piper_controller/max_delta_right"].value, [6,5,4,3,2,1])
            path.write_text("max_delta_left: [1,2,3,4,5,6]\n")
            config = self.load(["motion_limits_file:=" + str(path)])
            self.assertEqual(config.params["/vla_dual_piper_controller/max_delta_right"].value, [])

    def test_missing_or_malformed_file_rejects_expansion(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "limits.yaml"
            with self.assertRaises(roslaunch.loader.LoadException):
                self.load(["motion_limits_file:=" + str(path)])
            path.write_text("max_delta_left: [\n")
            with self.assertRaises(roslaunch.loader.LoadException):
                self.load(["motion_limits_file:=" + str(path)])

    def test_arrays_resolve_to_controller_private_parameters_in_both_modes(self):
        left = [0.001 * i for i in range(1, 7)]
        right = [0.002 * i for i in range(1, 7)]
        for mock in ("true", "false"):
            config = self.load(["max_delta_left:=" + str(left),
                                "max_delta_right:=" + str(right),
                                "mock_driver:=" + mock, "dry_run:=true",
                                "auto_enable:=false", "enable_cameras:=false"])
            for side, expected in (("left", left), ("right", right)):
                value = config.params["/vla_dual_piper_controller/max_delta_" + side].value
                self.assertEqual(value, expected)
                self.assertIsInstance(value, list)
            kinds = [node.type for node in config.nodes]
            self.assertEqual(kinds.count("vla_dual_mock_driver.py"), 2 if mock == "true" else 0)
            self.assertEqual(kinds.count("piper_ctrl_dual_feedback_node.py"), 0 if mock == "true" else 2)

    def test_expanded_arrays_are_validated_by_unchanged_controller(self):
        # Reuse the real Controller with the existing ROS/SDK/FK mock fixture.
        from test_vla_dual_piper_controller import DualControllerTests
        DualControllerTests.setUpClass()
        try:
            fixture = DualControllerTests()
            fixture.setUp()
            try:
                fixture.config.update({"~max_delta_left": [], "~max_delta_right": []})
                with self.assertRaises(ValueError):
                    fixture.module.VlaDualPiperController()
                for bad in ("[]", "[0.1]", "[0,0,0,0,0,0]",
                            "[-1,-1,-1,-1,-1,-1]", "[true,true,true,true,true,true]",
                            "[.nan,.nan,.nan,.nan,.nan,.nan]",
                            "[.inf,.inf,.inf,.inf,.inf,.inf]"):
                    for side in ("left", "right"):
                        args = {"left": "[0.01,0.02,0.03,0.04,0.05,0.06]",
                                "right": "[0.02,0.03,0.04,0.05,0.06,0.07]"}
                        args[side] = bad
                        config = self.load(["max_delta_" + key + ":=" + value
                                            for key, value in args.items()])
                        fixture.config.update({"~max_delta_" + key:
                            config.params["/vla_dual_piper_controller/max_delta_" + key].value
                            for key in args})
                        with self.subTest(side=side, bad=bad):
                            with self.assertRaises(ValueError):
                                fixture.module.VlaDualPiperController()
                config = self.load(["max_delta_left:=[0.01,0.02,0.03,0.04,0.05,0.06]",
                                    "max_delta_right:=[0.02,0.03,0.04,0.05,0.06,0.07]"])
                fixture.config.update({"~" + key.rsplit("/", 1)[1]: param.value
                                       for key, param in config.params.items()
                                       if key.startswith("/vla_dual_piper_controller/")})
                node = fixture.module.VlaDualPiperController()
                self.assertTrue(node.hold_requested)
                self.assertEqual(fixture.commands(node), [[], []])
            finally:
                fixture.doCleanups()
        finally:
            DualControllerTests.tearDownClass()


if __name__ == "__main__":
    unittest.main()
