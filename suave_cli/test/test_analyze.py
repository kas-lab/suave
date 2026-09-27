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

"""Tests for suave_cli.analyze."""

import argparse

from suave_cli import analyze, main
from suave_cli.docker_util import RUNNING_FORMAT


def test_analysis_body_forwards_arguments():
    body = analyze.analysis_body('wilcoxon_analysis', ['--ros-args', '-p', 'x:=a b'])
    assert body == "ros2 run suave_runner wilcoxon_analysis --ros-args -p 'x:=a b'"


def test_every_analysis_maps_to_a_console_script():
    assert {name: exe for name, (exe, _) in analyze.ANALYSES.items()} == {
        'wilcoxon': 'wilcoxon_analysis',
        'mann-whitney': 'mann_whitney_analysis',
        'summarize': 'summarize_results',
    }


def test_cmd_analyze_runs_in_target(make_ctx, suave_root, fake_runner):
    fake_runner.responses[('docker', 'inspect', '-f', RUNNING_FORMAT)] = (0, 'true\n')
    ctx = make_ctx(suave_root, args=argparse.Namespace(executable='summarize_results'),
                   passthrough=['--help'])
    assert analyze.cmd_analyze(ctx) == 0
    [argv] = fake_runner.find('docker', 'exec')
    assert argv[-1].endswith('{ ros2 run suave_runner summarize_results --help; }')


def test_analyze_help_lists_analyses(capsys):
    try:
        main.main(['analyze', '--help'], env={})
    except SystemExit:
        pass
    out = capsys.readouterr().out
    assert 'wilcoxon' in out
    assert 'mann-whitney' in out
