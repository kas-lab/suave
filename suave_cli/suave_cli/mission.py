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

"""suave mission: run one mission as a one-run suave_runner campaign."""

import argparse
import json
import shlex

from suave_cli.errors import CliError
from suave_cli.executor import join, Raw
from suave_cli.targets import run_in_target

MANAGERS = ('none', 'metacontrol', 'random', 'bt')
# Same readiness check as the bt experiment in runner_config.yml.
BT_REQUIRED_SERVICES = ['/f_maintain_motion/change_mode', '/f_generate_search_path/change_mode',
                        '/f_follow_pipeline/change_mode']

MISSION_EPILOG = """\
Starts ArduSub SITL, the simulation and the mission through the suave_runner
node, with one experiment and num_runs: 1. The runner waits for each part to
be ready, stops everything when the mission ends or on Ctrl+C, and writes a
normal campaign folder (CSV files, runs.json, logs, bag) under --result-path.
No perturbations are injected unless you set them with -p.

examples:
  suave mission                              no adaptation manager
  suave mission --manager bt --gui           BT manager, with the Gazebo GUI
  suave mission --manager bt -a bt_executable:=suave_bt_extended
  suave mission -p initial_pos_y:=2.5 -p experiment_logging:=true
"""


def experiment(manager, launch_args=()):
    """Return the runner experiment entry for one mission."""
    launch = join(['ros2', 'launch', 'suave_bringup', 'mission.launch.py',
                   f'adaptation_manager:={manager}', *launch_args])
    entry = {'experiment_launch': launch, 'num_runs': 1, 'adaptation_manager': manager,
             'mission_name': 'suave'}
    if manager == 'bt':
        entry['required_services'] = BT_REQUIRED_SERVICES
    return entry


def params_yaml(entry):
    """Return a runner params file holding only the experiments list."""
    return ('/suave_runner_node:\n  ros__parameters:\n    experiments:\n'
            f'      - |\n        {json.dumps(entry)}\n')


def mission_body(manager='none', launch_args=(), result_path=None, gui=False,
                 run_duration=None, params=()):
    """Return the shell code that runs one mission through suave_runner."""
    overrides = []
    if result_path:
        overrides.append(f'result_path:={result_path}')
    if gui:
        overrides.append('gui:=true')
    if run_duration is not None:
        overrides.append(f'run_duration:={run_duration}')
    overrides += list(params)
    parts = ['ros2', 'run', 'suave_runner', 'suave_runner', '--ros-args',
             '--params-file', Raw('"$params"')]
    for override in overrides:
        parts += ['-p', override]
    yaml = shlex.quote(params_yaml(experiment(manager, launch_args)))
    return ('params=$(mktemp --suffix=.yml) && trap \'rm -f "$params"\' EXIT && '
            f'printf %s {yaml} > "$params" && {join(parts)}')


def cmd_mission(ctx):
    """Run one mission."""
    args = ctx.args
    for value in args.launch_arg + args.param:
        if ':=' not in value:
            raise CliError(f'expected KEY:=VALUE, got {value!r}')
    return run_in_target(ctx, mission_body(args.manager, args.launch_arg, args.result_path,
                                           args.gui, args.run_duration, args.param))


def register(subparsers, common):
    """Add suave mission."""
    parser = subparsers.add_parser(
        'mission', parents=[common], help='run one mission (SITL, simulation and mission)',
        description='Run one SUAVE mission: a one-run suave_runner campaign.',
        epilog=MISSION_EPILOG, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--manager', choices=MANAGERS, default='none',
                        help='adaptation manager (default: none)')
    parser.add_argument('--gui', action='store_true', help='run with the Gazebo GUI')
    parser.add_argument('--result-path', metavar='PATH',
                        help='parent folder of the campaign (default: ~/suave/results)')
    parser.add_argument('--run-duration', type=int, metavar='SEC',
                        help='seconds to wait for the mission to finish (default: 600)')
    parser.add_argument('-a', '--launch-arg', action='append', default=[],
                        metavar='KEY:=VALUE',
                        help='mission.launch.py argument (repeatable)')
    parser.add_argument('-p', '--param', action='append', default=[], metavar='KEY:=VALUE',
                        help='suave_runner parameter override (repeatable)')
    parser.set_defaults(func=cmd_mission)
