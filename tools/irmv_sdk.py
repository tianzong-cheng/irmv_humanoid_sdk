#!/usr/bin/env python3
"""Start the configured IRMV SDK deployment from an installed prefix."""

import argparse
import json
import os
from pathlib import Path
import selectors
import signal
import subprocess
import sys
import tempfile

BIN = Path(__file__).resolve().parent
DEFAULT_CONFIG = '@IRMV_LAUNCH_CONFIG@'
SIMULATION_PYTHON = '@IRMV_SIMULATION_PYTHON@'
REAL_ENABLED = '@IRMV_ENABLE_REAL_BACKEND@' in ('ON', 'TRUE', '1')
VIEWER_FLAGS = ('viewer', 'viewer-follow', 'viewer-no-shadow')


class JsonNumber:
    def __init__(self, token):
        self.token = token


def snapshot_json(value):
    # C++ validates numeric tokens exactly. Do not round them through float.
    if isinstance(value, JsonNumber):
        return value.token
    if isinstance(value, dict):
        return '{' + ','.join(json.dumps(k) + ':' + snapshot_json(v)
                              for k, v in value.items()) + '}'
    if isinstance(value, list):
        return '[' + ','.join(map(snapshot_json, value)) + ']'
    return json.dumps(value)


def executable(path):
    if not path.is_file() or not os.access(path, os.X_OK):
        raise ValueError(f'missing executable: {path}')
    return str(path)


def with_studio(studio, command, environment, records_root):
    records_root.mkdir(parents=True, exist_ok=True)
    pending_signal = None

    def on_signal(signum, _frame):
        nonlocal pending_signal
        pending_signal = signum

    with tempfile.TemporaryDirectory(prefix='irmv-web-', dir='/tmp') as directory:
        socket = str(Path(directory) / 'live.sock')
        environment['IRMV_WEB_TELEMETRY_SOCKET'] = socket
        old_int = signal.signal(signal.SIGINT, on_signal)
        old_term = signal.signal(signal.SIGTERM, on_signal)
        service = None
        try:
            service = subprocess.Popen(
                [studio, '--records-root', str(records_root), '--data-dir', directory],
                stdout=subprocess.PIPE, text=True, start_new_session=True,
            )
            with selectors.DefaultSelector() as selector:
                selector.register(service.stdout, selectors.EVENT_READ)
                if not selector.select(timeout=10):
                    raise ValueError('Studio did not report readiness')
                url = service.stdout.readline().strip()
            if pending_signal is not None:
                return 128 + pending_signal
            if not url.startswith('http://127.0.0.1:') or service.poll() is not None:
                raise ValueError('Studio failed to start')
            print(f'Studio: {url}', flush=True)
            child = subprocess.Popen(command, env=environment)
            service_reported = False
            while True:
                if pending_signal is not None:
                    signum, pending_signal = pending_signal, None
                    child.send_signal(signum)
                try:
                    code = child.wait(timeout=0.25)
                    return code if code >= 0 else 128 - code
                except subprocess.TimeoutExpired:
                    if not service_reported and service.poll() is not None:
                        print('irmv-sdk: Studio stopped; SDK is still running',
                              file=sys.stderr, flush=True)
                        service_reported = True
        finally:
            try:
                if service is not None:
                    if service.poll() is None:
                        service.terminate()
                        try:
                            service.wait(timeout=3)
                        except subprocess.TimeoutExpired:
                            service.kill()
                            service.wait()
                    service.stdout.close()
            finally:
                signal.signal(signal.SIGINT, old_int)
                signal.signal(signal.SIGTERM, old_term)


def main():
    parser = argparse.ArgumentParser(prog='irmv-sdk', description=__doc__)
    parser.add_argument('--config', type=Path, help='override the deployment SDK config')
    parser.add_argument('--domain', type=int, help='override the configured DDS domain')
    parser.add_argument('--web', action='store_true', help='start the local Studio service')
    for flag in VIEWER_FLAGS:
        parser.add_argument('--' + flag, action='store_true', help='simulation viewer option')
    args = parser.parse_args()
    try:
        config = args.config or (BIN / DEFAULT_CONFIG if DEFAULT_CONFIG else None)
        if config is None or not config.is_file():
            raise ValueError(f'deployment config not found: {config or "(not configured)"}; provision it or pass --config')
        config = config.resolve()
        with config.open(encoding='utf-8') as stream:
            data = json.load(stream, parse_float=JsonNumber)
        if not isinstance(data, dict):
            raise ValueError(f'{config}: expected a JSON object')
        backend = data.get('backend')
        if backend not in ('fake', 'real', 'sim'):
            raise ValueError(f'{config}: expected backend fake, real, or sim')
        if backend == 'real' and not REAL_ENABLED:
            raise ValueError('this SDK installation does not include the Real backend')
        if REAL_ENABLED and args.config is None and backend != 'real':
            raise ValueError('configure a Real deployment or choose a config explicitly with --config')
        viewer = ['--' + flag for flag in VIEWER_FLAGS if getattr(args, flag.replace('-', '_'))]
        if viewer and backend != 'sim':
            raise ValueError('viewer options require the sim backend')
        common = ['--config', str(config), '--runtime', executable(BIN / 'robot_runtime'),
                  '--application', executable(BIN / 'application')]
        if args.domain is not None:
            common += ['--domain', str(args.domain)]
        if backend == 'sim':
            launcher = executable(BIN / 'sim_launch.py')
            if not SIMULATION_PYTHON:
                raise ValueError('set IRMV_SIMULATION_PYTHON when configuring the SDK installation')
            python = executable(BIN / SIMULATION_PYTHON)
            simulation = data.get('simulation')
            if not isinstance(simulation, dict):
                raise ValueError(f'{config}: expected a simulation object')
            run = Path(tempfile.mkdtemp(prefix='irmv-sim-', dir='/tmp'))
            simulation['socket_directory'] = str(run / 'run')
            snapshot = run / 'sdk.json'
            snapshot.write_text(snapshot_json(data), encoding='utf-8')
            common[1] = str(snapshot)
            print(f'Simulation evidence: {run / "run"}', flush=True)
            command = [launcher, *common, '--python', python, *viewer]
        else:
            command = [executable(BIN / 'sdk_launch.py'), *common]
        environment = os.environ.copy()
        environment.pop('IRMV_WEB_TELEMETRY_SOCKET', None)
        if args.web:
            telemetry = data.get('telemetry', {})
            if not isinstance(telemetry, dict):
                raise ValueError(f'{config}: invalid telemetry')
            recorder = telemetry.get('recorder', {})
            if not isinstance(recorder, dict):
                raise ValueError(f'{config}: invalid telemetry.recorder')
            if recorder.get('enabled') is not True:
                raise ValueError('--web requires telemetry.recorder.enabled=true in the deployment config')
            root = recorder.get('directory')
            if root is None:
                root = telemetry.get('directory', 'logs')
            if not isinstance(root, str) or not root:
                raise ValueError(f'{config}: invalid recorder directory')
            studio = executable(BIN / 'irmv_experiment_web')
            if backend == 'sim' and subprocess.run([*command, '--check-config']).returncode:
                raise ValueError('simulation config or asset validation failed')
            return with_studio(studio, command, environment, Path(root).resolve())
        os.execve(command[0], command, environment)
    except (OSError, ValueError) as error:
        print(f'irmv-sdk: {error}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130


if __name__ == '__main__':
    raise SystemExit(main())
