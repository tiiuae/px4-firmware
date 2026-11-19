"""Host-independent regression checks for Harmonic startup contracts."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

WORKSPACE = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(WORKSPACE / 'test/mavsdk_tests'))
from integration_test_runner import process_helper as ph
from integration_test_runner import test_runner as tr

session = ph.get_gz_session()


class GazeboStartupTests(unittest.TestCase):
    def test_paths_preserve_multiple_inherited_entries(self):
        with patch.dict(os.environ, {'GZ_SIM_RESOURCE_PATH': '/r1:/r2',
                                     'GZ_SIM_SYSTEM_PLUGIN_PATH': '/p1:/p2'}):
            runner = ph.Runner('/tmp', 'x500', 'case', False)
        runner.update_gz_sim_enviement(str(WORKSPACE), 'build/px4_sitl_default')
        self.assertTrue(runner.env['GZ_SIM_RESOURCE_PATH'].startswith('/r1:/r2:'))
        paths = runner.env['GZ_SIM_SYSTEM_PLUGIN_PATH'].split(':')
        self.assertEqual(paths[:2], ['/p1', '/p2'])
        self.assertTrue(any(p.endswith('/build_gz-sim_plugins') for p in paths))
        self.assertTrue(any(p.endswith('/src/modules/simulation/gz_plugins') for p in paths))
        self.assertTrue(Path(runner.env['GZ_SIM_SERVER_CONFIG_PATH']).is_file())

    def test_gui_and_headless(self):
        args = [str(WORKSPACE), '/tmp', 'x500', 'case', 1, False, 'build/px4_sitl_default']
        self.assertIn('--headless-rendering', ph.GzHarmonicServer(*args).args)
        self.assertNotIn('--headless-rendering', ph.GzHarmonicServer(*args, gui=True).args)

    def test_gui_resources_match_server_for_custom_build(self):
        args = [str(WORKSPACE), '/tmp', 'x500', 'case', False]
        client = ph.GzHarmonicClientRunner(*args, build_dir='build/custom')
        self.assertIn(str(WORKSPACE / 'Tools/simulation/gz/models'),
                      client.env['GZ_SIM_RESOURCE_PATH'].split(':'))
        self.assertIn(str(WORKSPACE / 'build/custom/src/modules/simulation/gz_plugins'),
                      client.env['GZ_SIM_SYSTEM_PLUGIN_PATH'].split(':'))

    def test_partition_explicit_and_new_sessions(self):
        self.assertEqual(session.select_partition({'GZ_PARTITION': 'operator'}), 'operator')
        self.assertNotEqual(session.select_partition({}), session.select_partition({}))

    @patch.object(session, 'services', return_value=[session.CREATE_SERVICE])
    def test_collision_rejected(self, _services):
        with self.assertRaisesRegex(RuntimeError, 'already advertised'):
            session.assert_unused({'GZ_PARTITION': 'operator'})

    def test_early_server_exit(self):
        process = Mock(returncode=3)
        process.poll.return_value = 3
        with self.assertRaisesRegex(RuntimeError, 'exited before'):
            session.wait_ready(process, {})

    @patch.object(session, 'services', return_value=[])
    def test_readiness_timeout(self, _services):
        process = Mock()
        process.poll.return_value = None
        with self.assertRaisesRegex(RuntimeError, 'Timed out'):
            session.wait_ready(process, {}, timeout=0.01)

    def test_spawn_checks_boolean_and_exit_status(self):
        for returncode, output in [(0, 'data: false'), (1, 'data: true'), (0, '')]:
            with self.subTest(returncode=returncode, output=output):
                with patch.object(session.subprocess, 'run', return_value=Mock(
                        returncode=returncode, stdout=output, stderr='')) as run:
                    with self.assertRaises(RuntimeError):
                        session.create_model('/tmp/a.sdf', 'drone', {'GZ_PARTITION': 'test'})
                    self.assertEqual(run.call_count, 1)
                    self.assertEqual(run.call_args.kwargs['env']['GZ_PARTITION'], 'test')

    @patch.object(session.subprocess, 'run', return_value=Mock(returncode=0, stdout='data: true\n'))
    def test_spawn_success(self, _run):
        session.create_model('/tmp/a.sdf', 'drone', {})

    def test_world_override_rejected(self):
        for args in [['other.sdf'], ['--world', 'other'], ['-g'], ['--server-config', 'other']]:
            with self.assertRaises(RuntimeError):
                session.validate_options(args)
        session.validate_options(['-s', '--headless-rendering', '-v', '3'])

    def test_spawn_not_retried(self):
        runner = Mock(spec=ph.GzHarmonicModelSpawnRunner)
        runner.has_started_ok.return_value = False
        tester = tr.Tester.__new__(tr.Tester)
        with patch.object(tr.time, 'sleep'):
            self.assertFalse(tester.try_to_run_several_times(runner))
        self.assertEqual(runner.start.call_count, 1)

    def test_partition_propagated_once_per_orchestrated_session(self):
        tester = tr.Tester.__new__(tr.Tester)
        tester.config = {'simulator': 'gz_sim', 'mode': 'sitl'}
        tester.build_dir = 'build/px4_sitl_default'
        tester.gui = True
        tester.verbose = False
        tester.speed_factor = 1
        tester.debugger = ''
        tester.tester_interface = Mock()
        tester.try_to_run_several_times = Mock(return_value=True)
        test = {'vehicle': 'ssrc_holybro_x500', 'model': 'ssrc_holybro_x500'}
        def fake(*args, **kwargs):
            runner = Mock()
            runner.env = {}
            runner.name = 'fake'
            return runner
        tester.tester_interface.create_test_runner.side_effect = fake
        with patch.object(ph, 'GzHarmonicServer', side_effect=fake), \
             patch.object(ph, 'GzHarmonicClientRunner', side_effect=fake), \
             patch.object(ph, 'Px4Runner', side_effect=fake), \
             patch.dict(os.environ, {}, clear=True):
            tester.start_runners('/tmp', test, 'case')
            first = {r.env['GZ_PARTITION'] for r in tester.active_runners}
            self.assertEqual(len(first), 1)
            tester.start_runners('/tmp', test, 'case')
            second = {r.env['GZ_PARTITION'] for r in tester.active_runners}
            self.assertTrue(first.isdisjoint(second))
            with patch.dict(os.environ, {'GZ_PARTITION': 'explicit'}):
                tester.start_runners('/tmp', test, 'case')
                self.assertEqual({r.env['GZ_PARTITION'] for r in tester.active_runners}, {'explicit'})

    def test_failed_launch_reaps_server(self):
        # Exercise real subprocess ownership and failure propagation without Gazebo installed.
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            fake = root / 'gz'
            fake.write_text('''#!/usr/bin/env python3
import os, pathlib, sys, time
root = pathlib.Path(os.environ['FAKE_ROOT'])
if sys.argv[1] == 'sim':
    (root / 'pid').write_text(str(os.getpid()))
    while True: time.sleep(1)
elif '-l' in sys.argv:
    if (root / 'pid').exists(): print('/world/default/create')
else: print('data: false')
''')
            fake.chmod(0o755)
            model = root / 'model.sdf'
            model.write_text('<sdf/>')
            env = dict(os.environ, PATH=str(root) + ':' + os.environ['PATH'],
                       FAKE_ROOT=folder, GZ_PARTITION='mock-test')
            result = subprocess.run([sys.executable, str(ph.HITL_SESSION_PATH), 'launch',
                                     '--model', str(model), '--world', str(model)],
                                    env=env, capture_output=True, text=True, timeout=15)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('Model creation failed', result.stderr)
            pid = int((root / 'pid').read_text())
            with self.assertRaises(ProcessLookupError):
                os.kill(pid, 0)


if __name__ == '__main__':
    unittest.main()
