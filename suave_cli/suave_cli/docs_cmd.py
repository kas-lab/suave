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

"""suave docs: build the Sphinx documentation the way CI does."""

import argparse
import posixpath

from suave_cli import term
from suave_cli.executor import join
from suave_cli.targets import in_checkout, run_in_target, select_target, target_checkout

DEFAULT_OUTPUT = 'docs/build'
# The SUAVE images have no Sphinx and their Python has no ensurepip (no venv
# pip), so the pinned docs requirements go into a --target folder that is
# reinstalled whenever docs/requirements.txt changes.
INSTALL_DEPS = (
    'deps="${XDG_CACHE_HOME:-$HOME/.cache}/suave/docs-python" && '
    'if ! cmp -s docs/requirements.txt "$deps/requirements.txt"; then '
    'echo "Installing docs/requirements.txt into $deps" >&2 && rm -rf "$deps" && '
    'python3 -m pip install --quiet --disable-pip-version-check --target "$deps" '
    '-r docs/requirements.txt && mkdir -p "$deps" && cp docs/requirements.txt "$deps/"; fi && '
    'export PYTHONPATH="$deps${PYTHONPATH:+:$PYTHONPATH}"')

DOCS_EPILOG = """\
Runs from the SUAVE checkout, with ROS and the workspace sourced (the API
pages import the packages), like CI:
  sphinx-build -W --keep-going -E -a docs/source OUTPUT

Sphinx and the other pinned docs/requirements.txt packages are installed on
first use into ~/.cache/suave/docs-python where the command runs. The
generated docs/source/_autosummary pages are removed first, so pages of
removed functions cannot fail the build. Arguments after '--' go to
sphinx-build.

examples:
  suave docs build
  suave docs build --output /tmp/suave-docs
  suave docs build -- -j auto
"""


def docs_build_body(checkout, output=DEFAULT_OUTPUT, extra=()):
    """Return the shell code that builds the docs from the checkout."""
    build = join(['python3', '-m', 'sphinx', '-W', '--keep-going', '-E', '-a',
                  'docs/source', output, *extra])
    return in_checkout(checkout, f'{INSTALL_DEPS} && rm -rf docs/source/_autosummary && '
                                 f'{build}')


def cmd_docs_build(ctx):
    """Build the documentation in the selected target."""
    target = select_target(ctx)
    checkout = target_checkout(ctx, target)
    output = ctx.args.output
    code = run_in_target(ctx, docs_build_body(checkout, output, ctx.passthrough), target)
    if code == 0 and not ctx.executor.dry_run:
        index = posixpath.join(checkout, output, 'index.html')
        where = f" in container '{target.container}'" if target.kind == 'docker' else ''
        term.info(f'Docs built: {index}{where}')
    return code


def register(subparsers, common):
    """Add suave docs."""
    parser = subparsers.add_parser(
        'docs', parents=[common], help='build the documentation',
        description='Build the SUAVE Sphinx documentation.',
        epilog=DOCS_EPILOG, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='docs_command', metavar='ACTION', required=True)
    build = sub.add_parser('build', parents=[common], help='build the HTML docs like CI',
                           description='Build the HTML docs with warnings as errors.',
                           epilog=DOCS_EPILOG,
                           formatter_class=argparse.RawDescriptionHelpFormatter)
    build.add_argument('--output', metavar='DIR', default=DEFAULT_OUTPUT,
                       help=f'output folder, relative to the checkout (default: {DEFAULT_OUTPUT})')
    build.set_defaults(func=cmd_docs_build, accepts_passthrough=True)
