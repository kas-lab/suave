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

"""Terminal output helpers: colors, messages and prompts."""

import os
import sys

_COLORS = {'red': '31', 'yellow': '33', 'green': '32', 'dim': '2', 'bold': '1'}


def use_color(stream):
    """Return True when ANSI colors should be written to stream."""
    if os.environ.get('NO_COLOR'):
        return False
    isatty = getattr(stream, 'isatty', None)
    return bool(isatty and isatty())


def style(text, color, stream=None):
    """Wrap text in an ANSI color when the stream supports it."""
    stream = sys.stderr if stream is None else stream
    if not use_color(stream):
        return text
    return f'\033[{_COLORS[color]}m{text}\033[0m'


def info(message):
    """Print an informational message to stderr."""
    print(message, file=sys.stderr)


def warn(message):
    """Print a yellow warning to stderr."""
    print(style(f'warning: {message}', 'yellow'), file=sys.stderr)


def error(message):
    """Print a red error to stderr."""
    print(style(f'error: {message}', 'red'), file=sys.stderr)


def command(text):
    """Print a command that is about to run, dimmed, to stderr."""
    print(style(f'+ {text}', 'dim'), file=sys.stderr)


def stdin_is_tty():
    """Return True when stdin is a terminal."""
    return sys.stdin.isatty()


def is_interactive():
    """Return True when both stdin and stdout are terminals."""
    return sys.stdin.isatty() and sys.stdout.isatty()


def ask_yes_no(question, default=False, input_fn=input):
    """Ask a yes/no question and return the answer."""
    suffix = ' [Y/n] ' if default else ' [y/N] '
    answer = input_fn(question + suffix).strip().lower()
    if not answer:
        return default
    return answer in ('y', 'yes')


def ask_value(question, default, input_fn=input):
    """Ask for a value and return default on empty input."""
    answer = input_fn(f'{question} [{default}]: ').strip()
    return answer if answer else default
