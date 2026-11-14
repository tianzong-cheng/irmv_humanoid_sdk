"""Launcher preparation contracts; no optional simulator imports or DDS peers."""
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))


class LauncherTest(unittest.TestCase):
    def test_launcher_available(self):
        self.assertTrue((ROOT / 'tools/sim_launch.py').is_file(),
                        'Task 9 independent launcher is missing')

    def test_exclusive_run_directory(self):
        from sim_launch import reserve_directory
        with tempfile.TemporaryDirectory(prefix='launcher paths with spaces ') as parent:
            path = Path(parent) / 'private run'
            reserve_directory(path)
            self.assertEqual(path.stat().st_mode & 0o777, 0o700)
            sentinel = path / 'bootstrap.json'
            sentinel.write_text('owned by previous run')
            with self.assertRaises(FileExistsError):
                reserve_directory(path)
            self.assertEqual(sentinel.read_text(), 'owned by previous run')

    def test_fresh_independent_ids(self):
        from sim_launch import identities
        a, b = identities(), identities()
        self.assertNotEqual(a, b)
        for pair in (a, b):
            self.assertNotEqual(pair['run_id'], pair['runtime_id'])
            self.assertTrue(all(len(bytes.fromhex(value)) == 16 and int(value, 16)
                                for value in pair.values()))


class OwnershipTest(unittest.TestCase):
    def fixture(self, directory, engine_factory=None):
        import json
        from sim_launch import Experiment
        from unittest.mock import patch
        sys.path.insert(0,str(ROOT/'tests/support/simulation'))
        from fake_engine import FakeEngine
        d=Path(directory)
        normalized=dict(socket_directory=str(d/'run'),asset_digest='b'*64,policy_path='unused',
            config_digest='a'*64,application_digest='c'*64,engine_digest='d'*64,
            physics_dt_us=1000,control_dt_us=2000,startup_timeout_us=2000000,
            phase_timeout_us=1000000,engine_config=dict(q=[0,0]))
        owner=Experiment(d/'config with spaces.json',d/'runtime with spaces',d/'app with spaces',
            engine_factory=engine_factory or FakeEngine,binding=lambda n:{})
        return owner,patch('sim_launch.subprocess.check_output',return_value=json.dumps(normalized))

    def test_engine_failure_starts_no_children(self):
        from unittest.mock import patch
        class BadEngine:
            def initialize(self,c):raise ValueError('startup sentinel')
            def close(self):pass
        with tempfile.TemporaryDirectory() as d:
            owner,normalize=self.fixture(d,BadEngine)
            with normalize,patch('sim_launch.subprocess.Popen') as spawn:
                try:
                    with self.assertRaisesRegex(RuntimeError,'startup sentinel'):owner.start()
                    spawn.assert_not_called()
                finally:owner.close()
            import json
            evidence=json.loads((Path(d)/'run/evidence.json').read_text())
            self.assertEqual(evidence['state'],'Failed')
            self.assertIn('startup sentinel',evidence['failure'])
            self.assertFalse(evidence['stop_proven'])

    def test_engine_ready_precedes_children_and_socket_collision(self):
        from unittest.mock import patch,Mock
        import socket
        with tempfile.TemporaryDirectory(prefix='launcher paths with spaces ') as d:
            owner,normalize=self.fixture(d)
            def spawn(argv,**kwargs):
                self.assertTrue(kwargs.get('start_new_session'), 'terminal signals must not hit SDK children')
                self.assertIsNotNone(owner.runner.coordinator.feedback)
                self.assertTrue((Path(d)/'run/bridge.sock').exists())
                self.assertEqual(argv[1:],[ '--config', str(Path(d)/'config with spaces.json')])
                p=Mock();p.poll.return_value=0;p.returncode=0
                return p
            with normalize,patch('sim_launch.subprocess.Popen',side_effect=spawn):
                try:
                    owner.start()
                    self.assertEqual(len(owner.children),2)
                    other,_=self.fixture(d)
                    sentinel=Path(d)/'run/evidence.json'
                    sentinel.write_text('previous evidence')
                    with self.assertRaises(FileExistsError):other.start()
                    other.close()
                    self.assertEqual(sentinel.read_text(),'previous evidence')
                finally:owner.close()
            # Runner bind collision never deletes the existing socket.
            from simulation.main import Runner
            sock=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET)
            sock.bind(str(Path(d)/'run/bridge.sock'))
            inode=(Path(d)/'run/bridge.sock').stat().st_ino
            collision=Runner(owner.runner.coordinator.b,Path(d)/'run/bridge.sock')
            self.assertEqual(collision.run(),1)
            self.assertEqual((Path(d)/'run/bridge.sock').stat().st_ino,inode)
            sock.close()

    def test_partial_spawn_cleanup_only_owned_children(self):
        from unittest.mock import patch,Mock
        with tempfile.TemporaryDirectory() as d:
            owner,normalize=self.fixture(d)
            child=Mock();child.poll.return_value=None
            with normalize,patch('sim_launch.subprocess.Popen',side_effect=[child,OSError('app spawn')]):
                try:
                    with self.assertRaisesRegex(OSError,'app spawn'):owner.start()
                finally:owner.close()
            child.terminate.assert_called_once();child.wait.assert_called_once()
            self.assertEqual(len(owner.children),1)

    def test_runner_and_both_child_codes_are_required(self):
        from unittest.mock import Mock
        from sim_launch import Experiment
        for runner_code,codes,proven in [(1,[0,0],True),(0,[0,3],True),(0,[1,0],True),(0,[0,0],False),(0,[0,0],True)]:
            owner=Experiment('config','runtime','app')
            owner.thread=Mock();owner.thread.is_alive.return_value=False
            owner.result=runner_code
            owner.runner=Mock()
            c=owner.runner.coordinator;c.state='Closed';c.failure='';c.stop_proven=proven
            c.runtime_only=True;c.release_state='Released';c.release_event_drops=0;c.release_events=[];c.step=10
            owner.children=[Mock(),Mock()]
            for child,code in zip(owner.children,codes):child.wait.return_value=code
            self.assertEqual(owner.wait(),0 if runner_code==0 and codes==[0,0] and proven else 1)

class ActualSdkTest(unittest.TestCase):
    def test_application_quit_retains_runtime_then_owner_closes(self):
        import json,subprocess,time
        from sim_launch import Experiment
        from simulation.main import Runner
        sys.path.insert(0,str(ROOT/'tests/support/simulation'))
        from fake_engine import FakeEngine
        from launch_fixture import prepare
        build=BUILD
        engines=[]
        class Engine(FakeEngine):
            def initialize(self,c):
                super().initialize(dict(q=[0,0]));self.closes=0;engines.append(self)
            def close(self):self.closes+=1;super().close()
        class Keys(Runner):
            def _actions(self,actions):
                for action in actions:
                    if action.kind=='send' and action.peer==3 and action.frame.kind==15:
                        key={60:b'd',70:b'i',120:b'l',150:b'x'}.get(action.frame.control_step)
                        if key:
                            owner.children[1].stdin.write(key);owner.children[1].stdin.flush()
                return super()._actions(actions)
        with tempfile.TemporaryDirectory(prefix='t9 sdk spaces ') as directory:
            d=Path(directory);config=prepare(ROOT,build,d)
            owner=Experiment(config,build/'robot_runtime',build/'application',engine_factory=Engine,
                binding=lambda n:{'fixture_only':True},runner_type=Keys,application_stdin=subprocess.PIPE)
            try:
                owner.start()
                end=time.monotonic()+30
                while owner.runner.coordinator.step<450 and time.monotonic()<end:
                    self.assertIsNone(owner.result,owner.runner.coordinator.failure)
                    time.sleep(.002)
                c=owner.runner.coordinator
                self.assertGreaterEqual(c.step,450)
                self.assertEqual(owner.children[1].wait(timeout=3),0)
                self.assertIsNone(owner.children[0].poll())
                self.assertTrue(c.runtime_only)
                self.assertEqual(c.states[-1]['execution_state'],2)
                self.assertEqual(c.release_state,'Released')
                owner.notify_shutdown()
                self.assertEqual(owner.wait(),0,c.failure)
                self.assertTrue(c.stop_proven)
                self.assertEqual(engines[0].closes,1)
            finally:owner.close()
            evidence=json.loads((d/'run/evidence.json').read_text())
            self.assertEqual(evidence['child_codes'],[0,0])
            self.assertTrue(evidence['engine_ready_before_sdk'])

if __name__ == '__main__':
    BUILD=Path(sys.argv.pop(1)).resolve() if len(sys.argv)>1 else ROOT/'build'
    unittest.main()
