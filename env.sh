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

# Put the suave CLI on PATH:  source /path/to/suave/env.sh  (bash only)

if [ -z "${BASH_VERSION:-}" ]; then
    echo "env.sh supports bash only; use the colcon workspace or symlink suave_cli/bin/suave" >&2
    return 1 2>/dev/null || exit 1
fi
if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    echo "source this file instead of executing it: source ${BASH_SOURCE[0]}" >&2
    exit 1
fi

SUAVE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
export SUAVE_ROOT
case ":${PATH}:" in
    *":${SUAVE_ROOT}/suave_cli/bin:"*) ;;
    *) export PATH="${SUAVE_ROOT}/suave_cli/bin:${PATH}" ;;
esac
