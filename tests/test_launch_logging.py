"""Exercise console reporting with actual ROS Humble process events."""

import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
RUNNER = r'''
import asyncio
import sys
from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, SetLaunchConfiguration
from ghand_launch_utils import logging_actions
# A signal-based watcher avoids threaded waitpid races in nested test runners.
asyncio.set_child_watcher(asyncio.SafeChildWatcher())
service = LaunchService(argv=['verbose:=' + sys.argv[3]], noninteractive=True)
process = ExecuteProcess(
    name=sys.argv[2],
    cmd=[sys.executable, '-c', 'import time; time.sleep(.2); ' + sys.argv[1]],
    output={'both': 'log'})
service.include_launch_description(LaunchDescription(
    [SetLaunchConfiguration('verbose', sys.argv[3])] + logging_actions() + [process]))
service.run()
'''


class LaunchLoggingTest(unittest.TestCase):
    def run_process(self, code, name="worker", verbose=False):
        with tempfile.TemporaryDirectory() as directory:
            env = dict(os.environ, ROS_LOG_DIR=directory)
            env["PYTHONPATH"] = str(ROOT / "ghand_launch_utils") + os.pathsep + env.get("PYTHONPATH", "")
            try:
                result = subprocess.run(
                    [sys.executable, "-c", RUNNER, code, name, str(verbose).lower()],
                    env=env, capture_output=True, text=True, timeout=15,
                )
            except subprocess.TimeoutExpired as error:
                logs = "\n".join(p.read_text(errors="replace") for p in Path(directory).rglob("*.log"))
                self.fail(f"Launch did not exit: {error.stdout!r}\n{error.stderr!r}\n{logs}")
            logs = "\n".join(p.read_text(errors="replace") for p in Path(directory).rglob("*.log"))
        self.assertEqual(result.returncode, 0, result.stderr)
        return result.stdout + result.stderr, logs

    def test_missing_serial_is_one_failure_and_preserves_full_log(self):
        output, logs = self.run_process(
            'import sys,time; sys.stderr.write("[WARN] serial connect failed: device=/dev/missing "'
            '"baud=1000000 error=No such file"); sys.stderr.flush(); time.sleep(.03); '
            'sys.stderr.write(" or directory\\n[ERROR] no slave\\n[FATAL] failed\\nStack trace\\n"); '
            'sys.stderr.flush(); sys.exit(1)', "ros2_control_node")
        self.assertEqual(output.count("Connection failed"), 1, output)
        self.assertIn("Serial port does not exist: /dev/missing", output)
        for noise in ("Stack trace", "no slave", "process started", "process has died"):
            self.assertNotIn(noise, output)
        self.assertIn("no slave", logs)
        self.assertIn("Stack trace", logs)

    def test_resource_errors_and_warnings_are_not_hidden(self):
        output, _ = self.run_process(
            'print("[ERROR] [rviz]: Mesh resource missing: hand.stl\\n" * 2, end=""); '
            'print("[WARN] [rviz]: Texture missing: hand.png")', "rviz2")
        self.assertEqual(output.count("Mesh resource missing: hand.stl"), 1, output)
        self.assertIn("Texture missing: hand.png", output)

    def test_partial_final_line_is_reported(self):
        output, _ = self.run_process('import sys; sys.stderr.write("[ERROR] plugin missing")')
        self.assertIn("plugin missing", output)

    def test_unstructured_failure_is_not_silent(self):
        output, logs = self.run_process('import sys; print("native failure"); sys.exit(3)')
        self.assertIn("exit code 3", output)
        self.assertIn("Full log:", output)
        self.assertIn("native failure", logs)

    def test_activation_status_is_visible(self):
        output, _ = self.run_process('print("[INFO] Successfully activated!")', "ros2_control_node")
        self.assertIn("Hardware active", output)
        self.assertNotIn("Connection failed", output)

    def test_verbose_restores_details(self):
        output, _ = self.run_process('print("[INFO] diagnostic detail")', verbose=True)
        self.assertIn("diagnostic detail", output)
        self.assertIn("process started", output)

    def test_runtime_error_and_native_exception_remain_visible(self):
        output, _ = self.run_process(
            'print("[INFO] Successfully activated!\\n"'
            '"[ERROR] [rs485]: RS485 mode write failed: error=Connection timed out\\n"'
            '"[ERROR] Hardware write failed on rs485 backend; commands will be frozen.\\n"'
            '"[ERROR] Entering error state; freezing commands.\\n"'
            '"  what():  parameter \'fingers\' has already been declared")',
            "ros2_control_node")
        self.assertIn("Hardware error", output)
        self.assertIn("Connection timed out", output)
        self.assertNotIn("Connection failed", output)
        self.assertNotIn("Entering error state", output)
        self.assertNotIn("Hardware write failed on", output)
        self.assertIn("Process error", output)
        self.assertIn("parameter 'fingers' has already been declared", output)

    def test_initial_failure_does_not_hide_a_later_exception(self):
        output, _ = self.run_process(
            'print("[ERROR] [GHandHardwareInterface]: Failed to initialize rs485 communication backend.\\n"'
            '"  what():  controller plugin could not load")', "ros2_control_node")
        self.assertIn("Connection failed", output)
        self.assertIn("controller plugin could not load", output)

    def test_hardware_fault_stops_before_controller_cascade(self):
        output, _ = self.run_process(
            'print("[INFO] Successfully activated!\\n"'
            '"[ERROR] [rs485]: RS485 mode write failed: error=Invalid exception code\\n"'
            '"[ERROR] [controller_manager]: State interface does not exist\\n"'
            '"[ERROR] [controller_manager]: Could not activate controller")',
            "ros2_control_node")
        self.assertEqual(output.count("Hardware error"), 1, output)
        self.assertIn("Invalid exception code", output)
        self.assertNotIn("State interface does not exist", output)
        self.assertNotIn("Could not activate controller", output)


if __name__ == "__main__":
    unittest.main()
