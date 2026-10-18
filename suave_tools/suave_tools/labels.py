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

"""Display labels for managing systems, shared by every result tool."""

MANAGERS = {  # key: (full label, short LaTeX label)
    'rebetmc': ('TacticalRetreat', 'TacticalRetreat'),
    'rosa_bt': ('ROSA-BT', 'ROSA-BT'),
    'bt': ('BT', 'BT'),
    'metacontrol': ('Metacontrol', 'MC'),
    'planta': ('PLANTA', 'PLANTA'),
    'random': ('Random', 'Random'),
    'none': ('None', 'None'),
}
MANAGER_ORDER = tuple(MANAGERS)


def label(key, short=False):
    """Return the display label of a manager; unknown keys are unchanged."""
    if key not in MANAGERS:
        return key
    return MANAGERS[key][1 if short else 0]
