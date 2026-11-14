# Copyright 2026 KAS Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for suave_cli.docs_cmd."""

import argparse
import subprocess

from suave_cli import docs_cmd
from suave_cli.docker_util import RUNNING_FORMAT

RUNNING = ('docker', 'inspect', '-f', RUNNING_FORMAT)


def fake_checkout(tmp_path):
    checkout = tmp_path / 'suave'
    (checkout / 'docs' / 'source' / '_autosummary').mkdir(parents=True)
    (checkout / 'docs' / 'requirements.txt').write_text('Sphinx==8.1.3\n')
    return checkout


def run_body(tmp_path, checkout, body):
    """Run body with fake python3 that logs its arguments and PYTHONPATH."""
    bin_dir = tmp_path / 'bin'
    bin_dir.mkdir(exist_ok=True)
    log = tmp_path / 'log'
    fake = bin_dir / 'python3'
    fake.write_text(f'#!/bin/bash\necho "$PYTHONPATH|$*" >> {log}\n')
    fake.chmod(0o755)
    env = {'PATH': f'{bin_dir}:/usr/bin:/bin', 'HOME': str(tmp_path / 'home'),
           'PYTHONPATH': '/ros'}
    result = subprocess.run(['bash', '-c', body], env=env, capture_output=True, text=True)
    return result, log.read_text().splitlines() if log.exists() else []


def test_body_builds_like_ci():
    body = docs_cmd.docs_build_body('/src/suave', 'docs/build', ['-j', 'auto'])
    assert 'cd /src/suave && ' in body
    assert body.endswith(
        'rm -rf docs/source/_autosummary && python3 -m sphinx -W --keep-going -E -a '
        'docs/source docs/build -j auto')


def test_body_installs_requirements_once(tmp_path):
    checkout = fake_checkout(tmp_path)
    body = docs_cmd.docs_build_body(str(checkout))
    result, calls = run_body(tmp_path, checkout, body)
    assert result.returncode == 0, result.stderr
    deps = tmp_path / 'home' / '.cache' / 'suave' / 'docs-python'
    assert calls[0].endswith(f'-m pip install --quiet --disable-pip-version-check '
                             f'--target {deps} -r docs/requirements.txt')
    assert calls[1] == f'{deps}:/ros|-m sphinx -W --keep-going -E -a docs/source docs/build'
    assert not (checkout / 'docs' / 'source' / '_autosummary').exists()
    _, calls = run_body(tmp_path, checkout, body)
    assert [call for call in calls if 'pip install' in call] == [calls[0]]
    (checkout / 'docs' / 'requirements.txt').write_text('Sphinx==9\n')
    _, calls = run_body(tmp_path, checkout, body)
    assert sum('pip install' in call for call in calls) == 2


def test_cmd_docs_build_uses_container_checkout(make_ctx, suave_root, fake_runner, capsys):
    fake_runner.responses[RUNNING] = (0, 'true\n')
    ctx = make_ctx(suave_root, args=argparse.Namespace(output='docs/build'))
    assert docs_cmd.cmd_docs_build(ctx) == 0
    [argv] = fake_runner.find('docker', 'exec')
    assert 'cd /home/ubuntu-user/suave_ws/src/suave && ' in argv[-1]
    assert '/home/ubuntu-user/suave_ws/src/suave/docs/build/index.html' in (
        capsys.readouterr().err)
