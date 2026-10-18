# Copyright 2026 KAS-lab
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

"""Tests for the shared managing-system labels."""

import pytest

from suave_tools.labels import label
from suave_tools.labels import MANAGER_ORDER
from suave_tools.labels import MANAGERS


@pytest.mark.parametrize('key, full, short', [
    ('rebetmc', 'TacticalRetreat', 'TacticalRetreat'),
    ('rosa_bt', 'ROSA-BT', 'ROSA-BT'),
    ('bt', 'BT', 'BT'),
    ('metacontrol', 'Metacontrol', 'MC'),
    ('planta', 'PLANTA', 'PLANTA'),
    ('random', 'Random', 'Random'),
    ('none', 'None', 'None'),
])
def test_known_labels(key, full, short):
    """Return the full label by default and the LaTeX label when short."""
    assert label(key) == full
    assert label(key, short=True) == short


def test_unknown_key_is_unchanged():
    """Leave unknown managers as their raw key in both forms."""
    assert label('my_manager') == 'my_manager'
    assert label('my_manager', short=True) == 'my_manager'


def test_order_follows_table():
    """Order managers as listed in the table."""
    assert MANAGER_ORDER == tuple(MANAGERS)
    assert MANAGER_ORDER[0] == 'rebetmc'
    assert MANAGER_ORDER[-1] == 'none'
