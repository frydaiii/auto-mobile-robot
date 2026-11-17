"""Exercise the repository runner in a relocated checkout without starting ROS."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


RUNNER = Path(__file__).resolve().parents[1] / '.agents/skills/run-amr-project/scripts/run.sh'


class RunSkillTest(unittest.TestCase):
    def test_relocated_checkout_and_argument_forwarding(self):
        self.assertTrue(RUNNER.is_file(), 'Repository runner is missing')
        with tempfile.TemporaryDirectory(prefix='amr checkout ') as directory:
            root = Path(directory)
            runner = root / '.agents/skills/run-amr-project/scripts/run.sh'
            runner.parent.mkdir(parents=True)
            shutil.copyfile(RUNNER, runner)
            workspace = root / 'ros2_ws'
            (workspace / 'install').mkdir(parents=True)
            (workspace / 'install/setup.bash').write_text('export AMR_TEST_OVERLAY=loaded\n')
            binary = root / 'bin'
            binary.mkdir()
            ros2 = binary / 'ros2'
            ros2.write_text('#!/bin/bash\nprintf "%s\\n" "$PWD" "$AMR_TEST_OVERLAY" "$QT_QPA_PLATFORM" "$@"\n')
            ros2.chmod(0o755)
            setup = root / 'ros setup.bash'
            setup.write_text(f'export PATH="{binary}:$PATH"\n')
            env = dict(os.environ, AMR_ROS_SETUP=str(setup))
            env.pop('AMR_WORKSPACE', None)
            for mode, args in [('launch', ['gui:=false', 'rviz:=false']), ('keyboard', [])]:
                result = subprocess.run(['bash', str(runner), mode, *args],
                                        cwd='/tmp', env=env, capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn(str(workspace), result.stdout)
                self.assertIn('loaded', result.stdout)
                if mode == 'launch':
                    self.assertIn('mapping.launch.py\ngui:=false\nrviz:=false', result.stdout)
                    self.assertIn('xcb', result.stdout)
                else:
                    self.assertIn('teleop_twist_keyboard', result.stdout)
                    self.assertIn('speed:=0.15', result.stdout)

    def test_invalid_mode_fails_without_starting_ros(self):
        result = subprocess.run(['bash', str(RUNNER), 'invalid'], capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)
        self.assertIn('Usage:', result.stderr)


if __name__ == '__main__':
    unittest.main()
