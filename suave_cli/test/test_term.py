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

"""Tests for suave_cli.term."""

from suave_cli import term


class FakeStream:

    def __init__(self, tty):
        self._tty = tty

    def isatty(self):
        return self._tty


def test_style_colors_only_on_tty(monkeypatch):
    monkeypatch.delenv('NO_COLOR', raising=False)
    assert term.style('x', 'red', FakeStream(True)) == '\033[31mx\033[0m'
    assert term.style('x', 'red', FakeStream(False)) == 'x'


def test_no_color_disables_colors(monkeypatch):
    monkeypatch.setenv('NO_COLOR', '1')
    assert term.style('x', 'yellow', FakeStream(True)) == 'x'


def test_messages_go_to_stderr(capsys):
    term.warn('careful')
    term.error('broken')
    captured = capsys.readouterr()
    assert captured.out == ''
    assert 'warning: careful' in captured.err
    assert 'error: broken' in captured.err


def test_ask_yes_no_defaults_to_no():
    assert term.ask_yes_no('q?', input_fn=lambda _p: '') is False
    assert term.ask_yes_no('q?', input_fn=lambda _p: 'Y') is True
    assert term.ask_yes_no('q?', default=True, input_fn=lambda _p: '') is True


def test_ask_value_keeps_default_on_empty():
    assert term.ask_value('k', 'dflt', input_fn=lambda _p: '  ') == 'dflt'
    assert term.ask_value('k', 'dflt', input_fn=lambda _p: 'new') == 'new'
