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

"""suave analyze: wrappers around the SUAVE analysis scripts."""

import argparse

from suave_cli.executor import join
from suave_cli.targets import in_checkout, run_in_target, select_target, target_checkout

ANALYSES = {
    'wilcoxon': ('wilcoxon_analysis', 'Wilcoxon signed-rank tests on matched campaigns'),
    'mann-whitney': ('mann_whitney_analysis', 'Mann-Whitney U tests on independent campaigns'),
    'summarize': ('summarize_results', 'summarize campaign results'),
}

BATCH = '/home/ubuntu-user/suave/results/batches/BATCH'

# Source-tree scripts, run by path with python3 from the SUAVE checkout, as the
# docs do. The suave_requirements scripts are not installed, and some defaults
# (footprint-alternatives' --output-dir) are relative to the checkout.
SCRIPTS = {
    'wilcoxon-batch': (
        'suave_runner/suave_runner/analysis/wilcoxon_analysis_batch.py',
        'Wilcoxon tests on every campaign of a batch', f'{BATCH} --correction holm'),
    'qq': (
        'suave_runner/suave_runner/analysis/qq_plot.py',
        'Q-Q plot of the paired differences of two columns',
        'results.csv --column-a suave --column-b baseline --output qq.png'),
    'qq-batch': (
        'suave_runner/suave_runner/analysis/qq_plot_batch.py',
        'Q-Q plots for every manager pair of a batch', BATCH),
    'latex-tables': (
        'suave_runner/suave_runner/latex/wilcoxon_latex_tables_batch.py',
        'LaTeX tables from the batch Wilcoxon results', BATCH),
    'requirements': (
        'suave_requirements/scripts/plot_requirement_fulfillment.py',
        'plot requirement fulfillment from recorded bags', BATCH),
    'representative-run': (
        'suave_requirements/scripts/export_representative_run_tikz.py',
        'export one representative run as PGFPlots tables', BATCH),
    'requirements-tikz': (
        'suave_requirements/scripts/export_requirement_fulfillment_tikz.py',
        'export requirement fulfillment as PGFPlots tables', BATCH),
    'footprint-alternatives': (
        'suave_requirements/scripts/export_search_footprint_alternatives.py',
        'export search-footprint summaries as PGFPlots tables', BATCH),
}

EXPERIMENTS = ('exp1', 'exp2', 'exp3', 'extended_exp1', 'extended_exp2', 'extended_exp3')

ANALYZE_EPILOG = """\
Arguments after '--' go to the analysis script unchanged. Paths are not
translated between the host and the container.

examples:
  suave analyze summarize -- --help
  suave analyze wilcoxon -- --ros-args --params-file my_analysis.yml
  suave analyze wilcoxon-exp exp1 /home/ubuntu-user/suave/results/CAMPAIGN --correction holm
  suave analyze wilcoxon-batch -- /home/ubuntu-user/suave/results/batches/BATCH
  suave analyze qq-batch -- --help
"""

SCRIPT_EPILOG = """\
Runs from the SUAVE checkout: python3 {path} <arguments after '--'>
Relative paths are relative to the checkout.

examples:
  suave analyze {name} -- {example}
  suave analyze {name} -- --help
"""

WILCOXON_EXP_EPILOG = """\
Runs: ros2 launch suave_runner EXPERIMENT_analysis_launch.py results_root:=RESULTS_ROOT
The launch file maps the campaign's CSV files for the bundled experiment
configurations. Extra launch arguments go after '--'.

examples:
  suave analyze wilcoxon-exp exp1 /home/ubuntu-user/suave/results/CAMPAIGN
  suave analyze wilcoxon-exp extended_exp2 /home/ubuntu-user/suave/results/CAMPAIGN \\
    --correction holm --output-root /home/ubuntu-user/suave/results/analysis
"""

SUMMARIZE_EPILOG = """\
Runs: ros2 run suave_runner summarize_results <results_path> [options]

The results path is a campaign folder containing aggregate *_suave.csv files,
for example bt_suave.csv, metacontrol_suave.csv, or another manager-specific
<managing_system>_suave.csv. Per-event CSV files are ignored.

Each aggregate CSV must contain these columns:
  pipeline found
  time searching pipeline (s)
  distance inspected (m)
  mean reaction time (s)

Pass summarize_results arguments after '--'. For example:
  CAMPAIGN=/home/ubuntu-user/suave/results/batches/BATCH/campaigns/exp3
  suave analyze summarize -- $CAMPAIGN
  suave analyze summarize -- $CAMPAIGN --latex
  suave analyze summarize -- $CAMPAIGN --latex-output /tmp/suave_results.tex

Useful summarize_results options:
  --latex              write suave_results.tex in the campaign folder
  --latex-output PATH  write the LaTeX table to PATH; also enables LaTeX output
  --precision N        decimal places for terminal and LaTeX values (default: 2)
"""


def analysis_body(executable, extra):
    """Return the shell code that runs one analysis script."""
    return join(['ros2', 'run', 'suave_runner', executable, *extra])


def script_body(checkout, script, extra):
    """Return the shell code that runs one source-tree script from the checkout."""
    return in_checkout(checkout, join(['python3', script, *extra]))


def wilcoxon_exp_body(experiment, results_root, correction=None, output_root=None,
                      extra=()):
    """Return the shell code that runs a bundled experiment's Wilcoxon launch file."""
    parts = ['ros2', 'launch', 'suave_runner', f'{experiment}_analysis_launch.py',
             f'results_root:={results_root}']
    if correction:
        parts.append(f'correction:={correction}')
    if output_root:
        parts.append(f'output_root:={output_root}')
    return join([*parts, *extra])


def cmd_analyze(ctx):
    """Run the selected analysis script in the selected target."""
    return run_in_target(ctx, analysis_body(ctx.args.executable, ctx.passthrough))


def cmd_analyze_script(ctx):
    """Run the selected source-tree script from the checkout in the selected target."""
    target = select_target(ctx)
    body = script_body(target_checkout(ctx, target), ctx.args.script, ctx.passthrough)
    return run_in_target(ctx, body, target)


def cmd_wilcoxon_exp(ctx):
    """Run the Wilcoxon launch file of one bundled experiment."""
    args = ctx.args
    return run_in_target(ctx, wilcoxon_exp_body(
        args.experiment, args.results_root, args.correction, args.output_root,
        ctx.passthrough))


def register(subparsers, common):
    """Add suave analyze."""
    parser = subparsers.add_parser(
        'analyze', parents=[common], help='run the suave_runner analysis scripts',
        description='Run a suave_runner analysis script where --exec says.',
        epilog=ANALYZE_EPILOG, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='analysis', metavar='ANALYSIS', required=True)
    for name, (executable, summary) in ANALYSES.items():
        kwargs = {'parents': [common], 'help': summary}
        if name == 'summarize':
            kwargs.update({
                'description': 'Summarize one SUAVE campaign results folder.',
                'epilog': SUMMARIZE_EPILOG,
                'formatter_class': argparse.RawDescriptionHelpFormatter,
            })
        leaf = sub.add_parser(name, **kwargs)
        leaf.set_defaults(func=cmd_analyze, executable=executable, accepts_passthrough=True)
    raw = argparse.RawDescriptionHelpFormatter
    exp = sub.add_parser(
        'wilcoxon-exp', parents=[common], help='Wilcoxon tests on a bundled experiment campaign',
        description='Run the Wilcoxon analysis launch file of a bundled experiment.',
        epilog=WILCOXON_EXP_EPILOG, formatter_class=raw)
    exp.add_argument('experiment', choices=EXPERIMENTS, metavar='EXPERIMENT',
                     help=f'bundled experiment: {", ".join(EXPERIMENTS)}')
    exp.add_argument('results_root', metavar='RESULTS_ROOT', help='the campaign folder')
    exp.add_argument('--correction', choices=('holm', 'none'),
                     help='multiple-comparison correction (launch default: none)')
    exp.add_argument('--output-root', metavar='DIR',
                     help='directory for output CSVs (default: RESULTS_ROOT)')
    exp.set_defaults(func=cmd_wilcoxon_exp, accepts_passthrough=True)
    for name, (script, summary, example) in SCRIPTS.items():
        leaf = sub.add_parser(
            name, parents=[common], help=summary,
            description=f'Run {script.rsplit("/", 1)[-1]} ({summary}).',
            epilog=SCRIPT_EPILOG.format(path=script, name=name, example=example),
            formatter_class=raw)
        leaf.set_defaults(func=cmd_analyze_script, script=script, accepts_passthrough=True)
