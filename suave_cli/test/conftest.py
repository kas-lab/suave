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

"""Shared pytest fixtures for the suave CLI tests."""

import argparse
from pathlib import Path
import subprocess
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from suave_cli import config  # noqa: E402
from suave_cli.context import AppContext  # noqa: E402
from suave_cli.executor import Executor  # noqa: E402

REPO = Path(__file__).resolve().parents[2]


class FakeRunner:
    """Stand-in for subprocess.run that records calls and replays canned results."""

    def __init__(self):
        self.calls = []
        self.responses = {}

    def __call__(self, argv, **kwargs):
        self.calls.append(list(argv))
        for prefix in sorted(self.responses, key=len, reverse=True):
            if tuple(argv[:len(prefix)]) == prefix:
                response = self.responses[prefix]
                if isinstance(response, BaseException):
                    raise response
                code, stdout = response
                return subprocess.CompletedProcess(argv, code, stdout, '')
        return subprocess.CompletedProcess(argv, 0, '', '')

    def find(self, *prefix):
        """Return the recorded calls that start with prefix."""
        return [call for call in self.calls if tuple(call[:len(prefix)]) == prefix]


@pytest.fixture
def fake_runner():
    return FakeRunner()


@pytest.fixture
def suave_root(tmp_path):
    root = tmp_path / 'ws' / 'src' / 'suave'
    (root / 'suave_cli').mkdir(parents=True)
    (root / 'docker').mkdir()
    (root / 'docker' / 'versions.env').write_text(
        '# pinned\nARDUSUB_COMMIT=abc\nARDUPILOT_GAZEBO_COMMIT="def"\n')
    (root / 'build_docker_images.sh').write_text('#!/bin/bash\n')
    return root


@pytest.fixture
def ros_setup(tmp_path):
    path = tmp_path / 'setup.bash'
    path.write_text('# fake ROS setup\n')
    return path


@pytest.fixture
def make_ctx(fake_runner):
    def factory(root, file_values=None, flags=None, env=None, container=False,
                dry_run=False, args=None, passthrough=(), interactive=False, answers=()):
        pending = list(answers)
        settings = config.resolve(flags or {}, env or {}, file_values or {})
        return AppContext(
            suave_root=root, in_container=container, interactive=interactive,
            settings=settings, executor=Executor(dry_run=dry_run, runner=fake_runner),
            args=args or argparse.Namespace(), passthrough=list(passthrough),
            config_path=config.config_path(root, container),
            input_fn=lambda _prompt: pending.pop(0))
    return factory
