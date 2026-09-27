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

"""suave analyze: wrappers around the suave_runner analysis scripts."""

import argparse

from suave_cli.executor import join
from suave_cli.targets import run_in_target

ANALYSES = {
    'wilcoxon': ('wilcoxon_analysis', 'Wilcoxon signed-rank tests on matched campaigns'),
    'mann-whitney': ('mann_whitney_analysis', 'Mann-Whitney U tests on independent campaigns'),
    'summarize': ('summarize_results', 'summarize campaign results'),
}

ANALYZE_EPILOG = """\
Arguments after '--' go to the analysis script unchanged.

examples:
  suave analyze summarize -- --help
  suave analyze wilcoxon -- --ros-args --params-file my_analysis.yml
"""


def analysis_body(executable, extra):
    """Return the shell code that runs one analysis script."""
    return join(['ros2', 'run', 'suave_runner', executable, *extra])


def cmd_analyze(ctx):
    """Run the selected analysis script in the selected target."""
    return run_in_target(ctx, analysis_body(ctx.args.executable, ctx.passthrough))


def register(subparsers, common):
    """Add suave analyze."""
    parser = subparsers.add_parser(
        'analyze', parents=[common], help='run the suave_runner analysis scripts',
        description='Run a suave_runner analysis script where --exec says.',
        epilog=ANALYZE_EPILOG, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='analysis', metavar='ANALYSIS', required=True)
    for name, (executable, summary) in ANALYSES.items():
        leaf = sub.add_parser(name, parents=[common], help=summary)
        leaf.set_defaults(func=cmd_analyze, executable=executable, accepts_passthrough=True)
