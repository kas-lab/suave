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

"""suave run, batch and campaign: wrappers around the suave_runner nodes."""

import argparse
import shlex

from suave_cli import term
from suave_cli.errors import CliError
from suave_cli.executor import join, Raw
from suave_cli.targets import run_in_target, select_target, target_results_dir

CAMPAIGN_GLOB = ('[0-9][0-9][0-9][0-9]_[0-9][0-9]_[0-9][0-9]_'
                 '[0-9][0-9]-[0-9][0-9]-[0-9][0-9]')
BATCH_SUMMARY = (
    'import json, sys; s = json.load(open(sys.argv[1])); o = s.get("campaign_order", []); '
    'd = sum(1 for n in o if s["campaigns"].get(n, {}).get("status") == "completed"); '
    'print(f"{sys.argv[1]}  {d}/{len(o)} campaigns completed")')

RUN_EPILOG = """\
examples:
  suave run                                  launch with the installed runner_config.yml
  suave run --config my_runner.yml           run with your own params file
  suave run --result-path ~/suave/results/x --seed 7 -p experiment_logging:=true
"""

BATCH_EPILOG = """\
examples:
  suave batch start                          launch with the installed batch_campaigns.yml
  suave batch start --config my_batch.yml --fail-fast
  suave batch resume --latest                resume the newest batch under the results dir
  suave batch resume ~/suave/results/batches/batch_20260926_101500/state.json
  suave batch list
"""

CAMPAIGN_EPILOG = """\
examples:
  suave campaign resume --latest --config my_runner.yml
  suave campaign resume ~/suave/results/2026_09_26_10-15-00 --config my_runner.yml
  suave campaign list
"""


def installed_config(name):
    """Return a shell expression for a config file installed with suave_runner."""
    return Raw(f'"$(ros2 pkg prefix --share suave_runner)/config/runner/{name}"')


def exists_guard(path):
    """Return shell code that stops with exit code 2 when path does not exist."""
    quoted = shlex.quote(path)
    return f'if [ ! -e {quoted} ]; then echo "not found: "{quoted} >&2; exit 2; fi; '


def _ros2_run(executable, params_file, overrides):
    parts = ['ros2', 'run', 'suave_runner', executable, '--ros-args']
    if params_file:
        parts += ['--params-file', params_file]
    for override in overrides:
        parts += ['-p', override]
    return join(parts)


def run_body(config_file=None, result_path=None, gui=False, seed=None, run_duration=None,
             params=()):
    """Return the shell code that runs one experiment campaign."""
    overrides = []
    if result_path:
        overrides.append(f'result_path:={result_path}')
    if gui:
        overrides.append('gui:=true')
    if seed is not None:
        overrides.append(f'random_seed:={seed}')
    if run_duration is not None:
        overrides.append(f'run_duration:={run_duration}')
    overrides += list(params)
    if config_file is None and not overrides:
        return join(['ros2', 'launch', 'suave_runner', 'suave_runner_launch.py'])
    guard = exists_guard(config_file) if config_file else ''
    params_file = config_file or installed_config('runner_config.yml')
    return guard + _ros2_run('suave_runner', params_file, overrides)


def batch_start_body(config_file=None, batch_dir=None, fail_fast=False, dry_run_batch=False):
    """Return the shell code that starts a batch."""
    overrides = []
    if batch_dir:
        overrides.append(f'batch_dir:={batch_dir}')
    if fail_fast:
        overrides.append('fail_fast:=true')
    if dry_run_batch:
        overrides.append('dry_run:=true')
    if config_file is None and not overrides:
        return join(['ros2', 'launch', 'suave_runner', 'run_batch_launch.py'])
    guard = exists_guard(config_file) if config_file else ''
    params_file = config_file or installed_config('batch_campaigns.yml')
    return guard + _ros2_run('run_batch', params_file, overrides)


def batch_resume_body(state_file):
    """Return the shell code that resumes a batch from its state.json."""
    return exists_guard(state_file) + _ros2_run(
        'run_batch', None, [f'resume_state_file:={state_file}'])


def campaign_resume_body(result_path, config_file=None):
    """Return the shell code that resumes a campaign into result_path."""
    guard = exists_guard(result_path) + (exists_guard(config_file) if config_file else '')
    params_file = config_file or installed_config('runner_config.yml')
    return guard + _ros2_run('suave_runner', params_file,
                             [f'resume_result_path:={result_path}'])


def latest_body(kind, results_dir):
    """Return shell code printing the newest batch state file or campaign folder."""
    base = shlex.quote(results_dir)
    if kind == 'batch':
        return f'ls -1d {base}/batches/batch_*/state.json 2>/dev/null | sort | tail -n 1'
    return f'ls -1d {base}/{CAMPAIGN_GLOB} 2>/dev/null | sort | tail -n 1'


def batch_list_body(results_dir):
    """Return shell code listing batches with their campaign progress."""
    base = shlex.quote(results_dir)
    return (f'for f in $(ls -1d {base}/batches/batch_*/state.json 2>/dev/null | sort -r); '
            f'do python3 -c {shlex.quote(BATCH_SUMMARY)} "$f"; done')


def campaign_list_body(results_dir):
    """Return shell code listing campaign folders with their finished runs."""
    base = shlex.quote(results_dir)
    return (f'for d in $(ls -1d {base}/{CAMPAIGN_GLOB} 2>/dev/null | sort -r); '
            'do echo "$d  $(ls "$d"/run_*_*.done 2>/dev/null | wc -l) runs done"; done')


def resolve_latest(ctx, target, kind):
    """Return the newest batch state file or campaign folder inside target."""
    results_dir = target_results_dir(ctx, target)
    code, out = ctx.executor.capture(target.argv(latest_body(kind, results_dir)))
    lines = out.strip().splitlines()
    if code or not lines:
        raise CliError(f'no {kind} found under {results_dir}')
    path = lines[-1].strip()
    term.info(f'Latest {kind}: {path}')
    return path


def _path_or_latest(ctx, target, path, kind):
    if bool(path) == bool(ctx.args.latest):
        raise CliError(f'give either a {kind} path or --latest')
    return resolve_latest(ctx, target, kind) if ctx.args.latest else path


def cmd_run(ctx):
    """Run one experiment campaign."""
    args = ctx.args
    body = run_body(args.config, args.result_path, args.gui, args.seed, args.run_duration,
                    args.param)
    return run_in_target(ctx, body)


def cmd_batch_start(ctx):
    """Start a batch of campaigns."""
    args = ctx.args
    body = batch_start_body(args.config, args.batch_dir, args.fail_fast, args.dry_run_batch)
    return run_in_target(ctx, body)


def cmd_batch_resume(ctx):
    """Resume a batch from its state.json or the newest one."""
    target = select_target(ctx)
    state_file = _path_or_latest(ctx, target, ctx.args.state_file, 'batch')
    return run_in_target(ctx, batch_resume_body(state_file), target)


def cmd_batch_list(ctx):
    """List batches with their progress."""
    target = select_target(ctx)
    return run_in_target(ctx, batch_list_body(target_results_dir(ctx, target)), target)


def cmd_campaign_resume(ctx):
    """Resume a campaign from its result folder or the newest one."""
    target = select_target(ctx)
    result_path = _path_or_latest(ctx, target, ctx.args.result_path, 'campaign')
    if not ctx.args.config:
        term.warn('no --config given: using the installed runner_config.yml, which must '
                  'match the config this campaign was started with')
    return run_in_target(ctx, campaign_resume_body(result_path, ctx.args.config), target)


def cmd_campaign_list(ctx):
    """List campaign folders with their finished runs."""
    target = select_target(ctx)
    return run_in_target(ctx, campaign_list_body(target_results_dir(ctx, target)), target)


def register(subparsers, common):
    """Add suave run, batch and campaign."""
    raw = argparse.RawDescriptionHelpFormatter
    run = subparsers.add_parser(
        'run', parents=[common], help='run an experiment campaign (suave_runner)',
        description='Run the suave_runner node. Without options this is the '
                    'suave_runner_launch.py launch file; options switch to ros2 run with '
                    'parameter overrides.', epilog=RUN_EPILOG, formatter_class=raw)
    run.add_argument('--config', metavar='FILE',
                     help='runner params file (default: installed runner_config.yml)')
    run.add_argument('--result-path', metavar='PATH', help='override result_path')
    run.add_argument('--gui', action='store_true', help='run with the Gazebo GUI')
    run.add_argument('--seed', type=int, help='override random_seed')
    run.add_argument('--run-duration', type=int, metavar='SEC', help='override run_duration')
    run.add_argument('-p', '--param', action='append', default=[], metavar='KEY:=VALUE',
                     help='extra ROS parameter override (repeatable)')
    run.set_defaults(func=cmd_run)

    batch = subparsers.add_parser(
        'batch', parents=[common], help='start, resume or list experiment batches',
        description='Start, resume or list batches of campaigns (run_batch node).',
        epilog=BATCH_EPILOG, formatter_class=raw)
    batch_sub = batch.add_subparsers(dest='batch_command', metavar='ACTION', required=True)
    start = batch_sub.add_parser('start', parents=[common], help='start a new batch')
    start.add_argument('--config', metavar='FILE',
                       help='batch params file (default: installed batch_campaigns.yml)')
    start.add_argument('--batch-dir', metavar='DIR', help='batch folder (default: automatic)')
    start.add_argument('--fail-fast', action='store_true',
                       help='stop at the first failed campaign')
    start.add_argument('--dry-run-batch', action='store_true',
                       help="only validate the campaigns (run_batch's dry_run parameter)")
    start.set_defaults(func=cmd_batch_start)
    resume = batch_sub.add_parser('resume', parents=[common], help='resume a batch')
    resume.add_argument('state_file', nargs='?', metavar='STATE_JSON',
                        help="the batch's state.json")
    resume.add_argument('--latest', action='store_true', help='resume the newest batch')
    resume.set_defaults(func=cmd_batch_resume)
    listing = batch_sub.add_parser('list', parents=[common],
                                   help='list batches with their progress')
    listing.set_defaults(func=cmd_batch_list)

    campaign = subparsers.add_parser(
        'campaign', parents=[common], help='resume or list experiment campaigns',
        description='Resume or list single campaigns (suave_runner result folders).',
        epilog=CAMPAIGN_EPILOG, formatter_class=raw)
    campaign_sub = campaign.add_subparsers(dest='campaign_command', metavar='ACTION',
                                           required=True)
    resume = campaign_sub.add_parser('resume', parents=[common], help='resume a campaign')
    resume.add_argument('result_path', nargs='?', metavar='RESULT_PATH',
                        help="the campaign's result folder")
    resume.add_argument('--latest', action='store_true', help='resume the newest campaign')
    resume.add_argument('--config', metavar='FILE',
                        help='the params file the campaign was started with')
    resume.set_defaults(func=cmd_campaign_resume)
    listing = campaign_sub.add_parser('list', parents=[common],
                                      help='list campaign folders with finished runs')
    listing.set_defaults(func=cmd_campaign_list)
