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
from pathlib import Path
import subprocess

from suave_cli import analyze, config, main
from suave_cli.docker_util import RUNNING_FORMAT

REPO = Path(__file__).resolve().parents[2]


def host_env(suave_root, ros_setup):
    config.save_file(config.config_path(suave_root, False), {'ros_setup': str(ros_setup)})
    return {'SUAVE_ROOT': str(suave_root), 'SUAVE_EXEC': 'host'}


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


def test_summarize_help_explains_passthrough_and_expected_results(capsys):
    try:
        main.main(['analyze', 'summarize', '--help'], env={})
    except SystemExit:
        pass
    out = capsys.readouterr().out
    assert "Pass summarize_results arguments after '--'" in out
    assert '*_suave.csv' in out
    assert 'pipeline found' in out
    assert '/home/ubuntu-user/suave/results' in out
    assert '--latex-output PATH' in out


def test_every_script_exists_in_the_checkout():
    for script, _, _ in analyze.SCRIPTS.values():
        assert (REPO / script).is_file(), script


def test_every_experiment_has_a_launch_file():
    for experiment in analyze.EXPERIMENTS:
        launch = REPO / 'suave_runner' / 'launch' / 'analysis' / f'{experiment}_analysis_launch.py'
        assert launch.is_file(), launch


def test_script_body_runs_from_checkout():
    body = analyze.script_body('/src/suave', 'a/qq_plot.py', ['x.csv', '--label-a', 'a b'])
    assert body == (
        'if [ ! -d /src/suave ]; then echo "SUAVE checkout not found: "/src/suave >&2; '
        "exit 2; fi; cd /src/suave && python3 a/qq_plot.py x.csv --label-a 'a b'")


def test_script_body_stops_when_checkout_is_missing(tmp_path):
    result = subprocess.run(
        ['bash', '-c', analyze.script_body(str(tmp_path / 'missing'), 'x.py', [])],
        capture_output=True, text=True)
    assert result.returncode == 2
    assert 'SUAVE checkout not found' in result.stderr


def test_script_uses_container_src_dir(make_ctx, suave_root, fake_runner):
    fake_runner.responses[('docker', 'inspect', '-f', RUNNING_FORMAT)] = (0, 'true\n')
    script = analyze.SCRIPTS['qq-batch'][0]
    ctx = make_ctx(suave_root, args=argparse.Namespace(script=script),
                   passthrough=['/r/BATCH'], file_values={'container_name': 'suave_x'})
    assert analyze.cmd_analyze_script(ctx) == 0
    [argv] = fake_runner.find('docker', 'exec')
    assert 'suave_x' in argv
    assert argv[-1].endswith(
        f'cd /home/ubuntu-user/suave_ws/src/suave && python3 {script} /r/BATCH; }}')


def test_script_uses_local_checkout(make_ctx, suave_root, ros_setup, fake_runner):
    ctx = make_ctx(suave_root, args=argparse.Namespace(script='s.py'), container=True,
                   file_values={'ros_setup': str(ros_setup)})
    assert analyze.cmd_analyze_script(ctx) == 0
    assert f'cd {suave_root} && python3 s.py; }}' in fake_runner.calls[-1][-1]


def test_wilcoxon_exp_body():
    assert analyze.wilcoxon_exp_body('exp1', '/r/c') == (
        'ros2 launch suave_runner exp1_analysis_launch.py results_root:=/r/c')
    assert analyze.wilcoxon_exp_body('exp2', '/r/c', 'holm', '/o', ['x:=1']) == (
        'ros2 launch suave_runner exp2_analysis_launch.py results_root:=/r/c '
        'correction:=holm output_root:=/o x:=1')


def test_dry_run_prints_script_command(suave_root, ros_setup, capsys):
    env = host_env(suave_root, ros_setup)
    code = main.main(['--dry-run', 'analyze', 'latex-tables', '--', '/b'], env=env)
    out = capsys.readouterr().out
    assert code == 0
    assert f'cd {suave_root} && python3 {analyze.SCRIPTS["latex-tables"][0]} /b' in out


def test_dry_run_wilcoxon_exp(suave_root, ros_setup, capsys):
    env = host_env(suave_root, ros_setup)
    code = main.main(['--dry-run', 'analyze', 'wilcoxon-exp', 'exp3', '/c',
                      '--correction', 'holm'], env=env)
    assert code == 0
    assert 'exp3_analysis_launch.py results_root:=/c correction:=holm' in (
        capsys.readouterr().out)


def test_script_help_shows_path_and_example(capsys):
    try:
        main.main(['analyze', 'footprint-alternatives', '--help'], env={})
    except SystemExit:
        pass
    out = capsys.readouterr().out
    assert 'python3 suave_requirements/scripts/export_search_footprint_alternatives.py' in out
    assert 'suave analyze footprint-alternatives -- /home/ubuntu-user/suave/results' in out
