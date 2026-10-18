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

"""Resolve batch and campaign folders for every result tool, without ROS."""

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Optional

from suave_tools.runs import RUNS_FILE

# New outputs only; old 'campaings_results' folders are left alone.
RESULTS_DIR = 'campaign_results'
LATEX_DIR = 'campaigns_latex_tables'
CAMPAIGNS_DIR = 'campaigns'
STATE_FILE = 'state.json'
MANIFEST_SCHEMA_VERSION = 2
MANIFEST_KEYS = ('name', 'package', 'config_file')


@dataclass
class BatchLayout:
    """A batch root, its campaign folders, and default output roots."""

    root: Path
    campaigns: list
    manifest: Optional[Path]
    results_root: Path
    latex_root: Path


def is_campaign(path):
    """Return whether a folder is a runner campaign (it has runs.json)."""
    return (Path(path) / RUNS_FILE).is_file()


def batch_layout(path):
    """Accept a batch dir, its campaigns/ dir, a campaign dir, or state.json."""
    path = Path(path).expanduser().resolve()
    if not path.exists():
        raise ValueError(f'not found: {path}')
    campaigns = None
    if path.is_file():
        if path.name != STATE_FILE:
            raise ValueError(f'not a batch manifest ({STATE_FILE}): {path}')
        root = path.parent
    elif is_campaign(path):
        # A campaign inside a batch shares the batch's output roots.
        parent = path.parent
        root = parent.parent if parent.name == CAMPAIGNS_DIR else parent
        campaigns = [path]
    elif (path / CAMPAIGNS_DIR).is_dir() or (path / STATE_FILE).is_file():
        root = path
    elif path.name == CAMPAIGNS_DIR:
        root = path.parent
    else:
        raise ValueError(
            f'{path}: expected a batch (with {CAMPAIGNS_DIR}/ or '
            f'{STATE_FILE}), its {CAMPAIGNS_DIR}/ directory, a campaign '
            f'(with {RUNS_FILE}), or a {STATE_FILE}')
    manifest = root / STATE_FILE
    manifest = manifest if manifest.is_file() else None
    if campaigns is None:
        if manifest is not None:
            load_manifest(manifest)
        folder = root / CAMPAIGNS_DIR
        campaigns = sorted(
            child for child in folder.iterdir() if is_campaign(child)
        ) if folder.is_dir() else []
    return BatchLayout(root=root, campaigns=campaigns, manifest=manifest,
                       results_root=root / RESULTS_DIR,
                       latex_root=root / LATEX_DIR)


def load_manifest(path):
    """Read a batch state.json, accepting only the schema-2 manifest."""
    path = Path(path)
    try:
        manifest = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f'cannot read state file {path}: {error}') from error
    if not isinstance(manifest, dict) or (
            manifest.get('schema_version') != MANIFEST_SCHEMA_VERSION):
        raise ValueError(f'unsupported state-file schema in {path}')
    campaigns = manifest.get('campaigns')
    if not isinstance(campaigns, list) or any(
            not isinstance(entry, dict) or
            any(key not in entry for key in MANIFEST_KEYS)
            for entry in campaigns):
        raise ValueError(f'invalid campaign list in {path}')
    return manifest
