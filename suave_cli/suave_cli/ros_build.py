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

"""suave build and suave test: colcon wrappers that run in the selected target."""

import argparse

from suave_cli.executor import join
from suave_cli.targets import run_in_target

DEFAULT_PACKAGES = (
    'suave', 'suave_base', 'suave_bringup', 'suave_bt', 'suave_metacontrol', 'suave_metrics',
    'suave_missions', 'suave_monitor', 'suave_msgs', 'suave_none', 'suave_random',
    'suave_runner', 'suave_tools')

BUILD_EPILOG = """\
examples:
  suave build                          build all SUAVE packages
  suave build suave_runner --clean     rebuild one package from scratch
  suave build -- --cmake-args -DCMAKE_BUILD_TYPE=Release
"""

TEST_EPILOG = """\
examples:
  suave test                           build and test all SUAVE packages
  suave test suave_runner --no-build   test one package without rebuilding
  suave test suave_monitor --lint      only linters (flake8, pep257, copyright, ...)
  suave test suave_runner -k wilcoxon  only pytest tests matching 'wilcoxon'
"""


def selected_packages(packages):
    """Return the requested packages, or all SUAVE packages when none are given."""
    return list(packages) or list(DEFAULT_PACKAGES)


def build_body(pkgs, clean=False, extra=()):
    """Return the shell code that builds pkgs."""
    parts = []
    if clean:
        dirs = [path for pkg in pkgs for path in (f'build/{pkg}', f'install/{pkg}')]
        parts.append(join(['rm', '-rf', *dirs]))
    parts.append(join(['colcon', 'build', '--symlink-install', '--packages-select', *pkgs,
                       *extra]))
    return ' && '.join(parts)


def test_body(pkgs, build=True, lint=False, keyword=None, extra=()):
    """Return the shell code that tests pkgs and fails if any of them failed."""
    colcon = ['colcon', 'test', '--packages-select', *pkgs, '--event-handlers',
              'console_direct+']
    pytest_args = (['-m', 'linter'] if lint else []) + (['-k', keyword] if keyword else [])
    if pytest_args:
        colcon += ['--pytest-args', *pytest_args]
    if lint:
        colcon += ['--ctest-args', '-L', 'linter']
    colcon += list(extra)
    results = (
        f'for p in {join(pkgs)}; do '
        'if [ ! -d "build/$p" ]; then echo "no test results for $p (not built)" >&2; '
        'rc=1; continue; fi; '
        'colcon test-result --verbose --test-result-base "build/$p" || rc=1; '
        'done; exit $rc')
    body = f'rc=0; {join(colcon)} || rc=1; {results}'
    if build:
        body = f'{build_body(pkgs)} || exit $?; {body}'
    return body


def cmd_build(ctx):
    """Run colcon build in the selected target."""
    pkgs = selected_packages(ctx.args.packages)
    return run_in_target(ctx, build_body(pkgs, ctx.args.clean, ctx.passthrough))


def cmd_test(ctx):
    """Run colcon test and per-package test-result in the selected target."""
    args = ctx.args
    body = test_body(selected_packages(args.packages), not args.no_build, args.lint,
                     args.keyword, ctx.passthrough)
    return run_in_target(ctx, body)


def register(subparsers, common):
    """Add suave build and suave test."""
    raw = argparse.RawDescriptionHelpFormatter
    build = subparsers.add_parser(
        'build', parents=[common], help='colcon build SUAVE packages',
        description='Run colcon build --symlink-install where --exec says.',
        epilog=BUILD_EPILOG, formatter_class=raw)
    build.add_argument('packages', nargs='*', metavar='PKG',
                       help='packages to build (default: all SUAVE packages)')
    build.add_argument('--clean', action='store_true',
                       help='delete build/PKG and install/PKG first')
    build.set_defaults(func=cmd_build, accepts_passthrough=True)

    test = subparsers.add_parser(
        'test', parents=[common], help='build and test SUAVE packages',
        description='Build, then colcon test, then check colcon test-result per package. '
                    'The exit code is non-zero if any selected package failed.',
        epilog=TEST_EPILOG, formatter_class=raw)
    test.add_argument('packages', nargs='*', metavar='PKG',
                      help='packages to test (default: all SUAVE packages)')
    test.add_argument('--no-build', action='store_true', help='skip the build step')
    test.add_argument('--lint', action='store_true', help='run only linter tests')
    test.add_argument('-k', dest='keyword', metavar='EXPR',
                      help='run only pytest tests matching EXPR')
    test.set_defaults(func=cmd_test, accepts_passthrough=True)
