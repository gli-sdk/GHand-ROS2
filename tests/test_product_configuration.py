"""Offline contracts for package-internal product configuration isolation."""

import ast
from pathlib import Path
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]


class ProductConfigurationTest(unittest.TestCase):
    def test_lite1_product_configuration_consistency(self):
        facts = yaml.safe_load((ROOT / "ghand_description/config/ghandlite1.yaml").read_text(encoding="utf-8"))
        controllers = yaml.safe_load((ROOT / "ghand_bringup/config/ghandlite1/controllers.yaml").read_text(encoding="utf-8"))
        self.assertEqual(controllers["joint_status_broadcaster"]["ros__parameters"]["joints"], facts["joints"])
        self.assertEqual(controllers["joint_trajectory_controller"]["ros__parameters"]["joints"], facts["active_joints"])
        self.assertEqual(controllers["tactile_broadcaster"]["ros__parameters"]["point_counts"], facts["tactile_region_counts"])
        moveit = ROOT / "ghand_moveit_config/config/ghandlite1"
        mapping = yaml.safe_load((moveit / "moveit_controllers.yaml").read_text(encoding="utf-8"))
        self.assertEqual(mapping["moveit_simple_controller_manager"]["joint_trajectory_controller"]["joints"], facts["active_joints"])
        for side in ("left", "right"):
            srdf = ET.parse(moveit / f"ghandlite1_{side}.srdf").getroot()
            hand = srdf.find("group[@name='hand']")
            self.assertEqual({joint.get("name") for joint in hand.findall("joint")}, set(facts["active_joints"]))
            self.assertEqual({joint.get("name") for joint in srdf.findall("passive_joint")}, set(facts["joints"]) - set(facts["active_joints"]))

    def test_controller_and_moveit_joint_sets_match(self):
        config = ROOT / "ghand_bringup/config/ghand5"
        controllers = yaml.safe_load((config / "controllers.yaml").read_text(encoding="utf-8"))
        commanded = set(controllers["joint_trajectory_controller"]["ros__parameters"]["joints"])
        moveit = ROOT / "ghand_moveit_config/config/ghand5"
        mapped = yaml.safe_load((moveit / "moveit_controllers.yaml").read_text(encoding="utf-8"))
        self.assertEqual(commanded, set(mapped["moveit_simple_controller_manager"]["joint_trajectory_controller"]["joints"]))
        for side in ("left", "right"):
            srdf = ET.parse(moveit / f"ghand5_{side}.srdf").getroot()
            hand = srdf.find("group[@name='hand']")
            self.assertEqual(commanded, {joint.get("name") for joint in hand.findall("joint")})

    def test_all_launch_files_declare_product_and_keep_default(self):
        for package in ("ghand_bringup", "ghand_gazebo", "ghand_moveit_config"):
            for path in (ROOT / package / "launch").glob("*.launch.py"):
                with self.subTest(path=path):
                    tree = ast.parse(path.read_text(encoding="utf-8"))
                    declarations = [node for node in ast.walk(tree)
                                    if isinstance(node, ast.Call)
                                    and isinstance(node.func, ast.Name)
                                    and node.func.id == "DeclareLaunchArgument"
                                    and node.args and isinstance(node.args[0], ast.Constant)
                                    and node.args[0].value == "product_name"]
                    self.assertEqual(len(declarations), 1)
                    values = {kw.arg: ast.literal_eval(kw.value) for kw in declarations[0].keywords
                              if isinstance(kw.value, ast.Constant)}
                    self.assertEqual(values["default_value"], "ghand5")
                    choices = next(kw.value for kw in declarations[0].keywords if kw.arg == "choices")
                    expression = compile(ast.Expression(choices), str(path), "eval")
                    self.assertEqual(eval(expression, {
                        "Path": Path,
                        "get_package_share_directory": lambda name: str(ROOT / name),
                    }), ["ghand5", "ghandlite1"])

    def test_dual_moveit_generator_uses_selected_product_data(self):
        moveit = ROOT / "ghand_moveit_config"
        with tempfile.TemporaryDirectory() as tmp:
            outputs = [Path(tmp) / name for name in ("dual.srdf", "limits.yaml", "kinematics.yaml")]
            subprocess.run([
                sys.executable, str(moveit / "scripts/generate_dual_moveit_config.py"),
                "--left-srdf", str(moveit / "config/ghand5/ghand5_left.srdf"),
                "--right-srdf", str(moveit / "config/ghand5/ghand5_right.srdf"),
                "--joint-limits", str(moveit / "config/ghand5/joint_limits.yaml"),
                "--kinematics", str(moveit / "config/ghand5/kinematics.yaml"),
                "--srdf-output", str(outputs[0]),
                "--joint-limits-output", str(outputs[1]),
                "--kinematics-output", str(outputs[2]),
            ], check=True, capture_output=True, text=True)
            self.assertEqual(ET.parse(outputs[0]).getroot().get("name"), "ghand5-dual")
            virtual = ET.parse(outputs[0]).getroot().find("virtual_joint")
            self.assertEqual(virtual.get("child_link"), "world")
            source_limits = yaml.safe_load((moveit / "config/ghand5/joint_limits.yaml").read_text(encoding="utf-8"))
            dual_limits = yaml.safe_load(outputs[1].read_text(encoding="utf-8"))
            expected = {prefix + name: value for prefix in ("left_", "right_")
                        for name, value in source_limits["joint_limits"].items()}
            self.assertEqual(dual_limits["joint_limits"], expected)

    def test_dual_generator_does_not_assume_ghand5_joint_or_group_names(self):
        moveit = ROOT / "ghand_moveit_config"
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            for side in ("left", "right"):
                srdf = (moveit / f"config/ghand5/ghand5_{side}.srdf").read_text(encoding="utf-8")
                (tmp / f"{side}.srdf").write_text(
                    srdf.replace("ghand5-", "testhand-").replace("Thumb_MCP", "test_joint"),
                    encoding="utf-8",
                )
            (tmp / "limits.yaml").write_text("joint_limits:\n  test_joint:\n    max_velocity: 7.0\n")
            (tmp / "kinematics.yaml").write_text("test_group:\n  kinematics_solver: test_solver\n")
            subprocess.run([
                sys.executable, str(moveit / "scripts/generate_dual_moveit_config.py"),
                "--left-srdf", str(tmp / "left.srdf"), "--right-srdf", str(tmp / "right.srdf"),
                "--joint-limits", str(tmp / "limits.yaml"), "--kinematics", str(tmp / "kinematics.yaml"),
                "--srdf-output", str(tmp / "dual.srdf"),
                "--joint-limits-output", str(tmp / "dual_limits.yaml"),
                "--kinematics-output", str(tmp / "dual_kinematics.yaml"),
            ], check=True, capture_output=True, text=True)
            self.assertEqual(ET.parse(tmp / "dual.srdf").getroot().get("name"), "testhand-dual")
            self.assertEqual(yaml.safe_load((tmp / "dual_limits.yaml").read_text()), {
                "joint_limits": {"left_test_joint": {"max_velocity": 7.0},
                                 "right_test_joint": {"max_velocity": 7.0}},
            })
            self.assertEqual(set(yaml.safe_load((tmp / "dual_kinematics.yaml").read_text())),
                             {"left_test_group", "right_test_group"})


try:
    import xacro
except ImportError:
    xacro = None


@unittest.skipIf(xacro is None, "Install xacro to check resource expansion")
class XacroConfigurationTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        description = Path(self.tmp.name)
        (description / "config").mkdir()
        (description / "config/ghandlite1.yaml").write_text(
            (ROOT / "ghand_description/config/ghandlite1.yaml").read_text(encoding="utf-8"),
            encoding="utf-8",
        )
        for product in ("ghand5", "ghandlite1"):
            (description / "urdf" / product).mkdir(parents=True)
            for side in ("left", "right", "dual"):
                (description / f"urdf/{product}/{product}_{side}_description.urdf").write_text(
                    f'<robot name="{product}-{side}"><link name="base_link"/></robot>'
                )
        shares = {name: str(ROOT / name) for name in ("ghand_bringup", "ghand_gazebo")}
        shares["ghand_description"] = str(description)
        packages = types.ModuleType("ament_index_python.packages")
        packages.get_package_share_directory = lambda name: shares[name]
        self.modules = patch.dict(sys.modules, {
            "ament_index_python": types.ModuleType("ament_index_python"),
            "ament_index_python.packages": packages,
        })
        self.modules.start()
        self.addCleanup(self.modules.stop)

    def expand(self, package, filename, **mappings):
        return ET.fromstring(xacro.process_file(
            str(ROOT / package / "config" / filename), mappings=mappings
        ).toxml())

    def test_single_mock_defaults_and_commanded_joint_contract(self):
        robot = self.expand("ghand_bringup", "ghand_hardware_description.urdf.xacro",
                            use_mock_hardware="true")
        self.assertEqual(robot.get("name"), "ghand5-right")
        system = robot.find("ros2_control")
        self.assertEqual(system.findtext("hardware/plugin"), "mock_components/GenericSystem")
        self.assertEqual(len(system.findall("joint")), 18)
        self.assertEqual(len([joint for joint in system.findall("joint")
                              if joint.find("command_interface") is not None]), 13)

    def test_real_profile_overrides_and_product_tactile_layout(self):
        robot = self.expand("ghand_bringup", "ghand_hardware_description.urdf.xacro",
                            hand_side="left", use_mock_hardware="false", device="/dev/testhand",
                            profile_file=str(ROOT / "ghand_bringup/config/profiles/rs485_left.yaml"))
        system = robot.find("ros2_control")
        params = {param.get("name"): param.text for param in system.findall("hardware/param")}
        self.assertEqual(params["device"], "/dev/testhand")
        self.assertEqual(params["communication_type"], "rs485")
        self.assertEqual(params["tactile_region_counts"], "38,31,31,31,31")
        self.assertEqual(len(system.findall("sensor")), 6)

    def test_dual_mock_resources_match_controller_names(self):
        robot = self.expand("ghand_bringup", "ghand_dual_hardware_description.urdf.xacro")
        systems = robot.findall("ros2_control")
        self.assertEqual(len(systems), 2)
        controllers = yaml.safe_load((ROOT / "ghand_bringup/config/ghand5/dual_controllers.yaml").read_text(encoding="utf-8"))
        for side, system in zip(("left", "right"), systems):
            commanded = {joint.get("name") for joint in system.findall("joint")
                         if joint.find("command_interface") is not None}
            self.assertEqual(commanded, set(controllers[f"{side}_joint_trajectory_controller"]["ros__parameters"]["joints"]))

    def test_gazebo_single_and_dual_plugin_paths_exist(self):
        for filename in ("ghand_gazebo_description.urdf.xacro", "ghand_dual_gazebo_description.urdf.xacro"):
            robot = self.expand("ghand_gazebo", filename)
            for path in robot.findall("gazebo/plugin/parameters"):
                self.assertTrue(Path(path.text).is_file(), path.text)
            self.assertEqual(len(robot.findall("ros2_control/joint")), 36 if "dual" in filename else 18)

    def test_unknown_product_does_not_fall_back_to_ghand5(self):
        with self.assertRaises(xacro.XacroException):
            self.expand("ghand_bringup", "ghand_hardware_description.urdf.xacro",
                        product_name="unknown", use_mock_hardware="true")

    def test_lite1_resources_and_capabilities(self):
        for mock in ("true", "false"):
            robot = self.expand("ghand_bringup", "ghand_hardware_description.urdf.xacro",
                                product_name="ghandlite1", use_mock_hardware=mock,
                                profile_file=str(ROOT / "ghand_bringup/config/profiles/ethercat_default.yaml"))
            system = robot.find("ros2_control")
            self.assertEqual(len(system.findall("joint")), 11)
            commanded = {joint.get("name") for joint in system.findall("joint")
                         if joint.find("command_interface") is not None}
            self.assertEqual(commanded, {"THUMB_TMC_FE", "THUMB_TMC_AA", "FF_MCP", "MF_MCP", "RF_MCP", "LF_MCP"})
            self.assertFalse(system.findall("joint/command_interface[@name='loop_type']"))
            self.assertFalse(system.findall("joint/command_interface[@name='kp']"))
            self.assertFalse(system.findall("joint/command_interface[@name='kd']"))
            if mock == "false":
                self.assertEqual(system.findtext("hardware/param[@name='product_name']"), "ghandlite1")
                counts = [len(sensor.findall("state_interface")) for sensor in system.findall("sensor")[:5]]
                self.assertEqual(counts, [3 + 38 * 3] + [3 + 31 * 3] * 4)

    def test_lite1_dual_and_gazebo_resources(self):
        for package, filename in (
            ("ghand_bringup", "ghand_dual_hardware_description.urdf.xacro"),
            ("ghand_gazebo", "ghand_dual_gazebo_description.urdf.xacro"),
            ("ghand_gazebo", "ghand_gazebo_description.urdf.xacro"),
        ):
            robot = self.expand(package, filename, product_name="ghandlite1")
            self.assertEqual(len(robot.findall("ros2_control/joint")), 22 if "dual" in filename else 11)


if __name__ == "__main__":
    unittest.main()
