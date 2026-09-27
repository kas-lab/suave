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

"""Run commands locally, honouring --dry-run and --verbose."""

import shlex
import subprocess

from suave_cli import term
from suave_cli.errors import CliError


class Raw(str):
    """A shell fragment that must not be quoted, such as a $(...) substitution."""


def join(parts):
    """Join command parts into one shell string, quoting all but Raw parts."""
    return ' '.join(part if isinstance(part, Raw) else shlex.quote(str(part)) for part in parts)


class Executor:
    """Run commands, or only print them in dry-run mode."""

    def __init__(self, dry_run=False, verbose=False, runner=None):
        self.dry_run = dry_run
        self.verbose = verbose
        self.runner = runner or subprocess.run

    def run(self, argv, cwd=None, env=None):
        """Run argv attached to the terminal and return its exit code."""
        text = join(argv)
        if cwd is not None:
            text = f'(cd {shlex.quote(str(cwd))} && {text})'
        if self.dry_run:
            print(text)
            return 0
        if self.verbose:
            term.command(text)
        try:
            return self.runner([str(part) for part in argv], cwd=cwd, env=env).returncode
        except FileNotFoundError:
            raise CliError(f'command not found: {argv[0]}', code=127) from None

    def capture(self, argv):
        """Run a read-only query, also in dry-run mode, and return (code, stdout)."""
        if self.verbose:
            term.command(join(argv))
        try:
            result = self.runner([str(part) for part in argv], capture_output=True, text=True)
        except FileNotFoundError:
            return 127, ''
        return result.returncode, result.stdout
