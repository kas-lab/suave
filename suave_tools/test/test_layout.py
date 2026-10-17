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

"""Test batch and campaign layout resolution."""

import json

import pytest

from suave_tools.layout import batch_layout
from suave_tools.layout import load_manifest


def _campaign(path):
    path.mkdir(parents=True)
    (path / 'runs.json').write_text(json.dumps(
        {'schema_version': 2, 'campaign': {}, 'runs': {}}))
    return path


def _batch(root, names=('exp2', 'exp1')):
    campaigns = [_campaign(root / 'campaigns' / name) for name in names]
    (root / 'campaigns' / 'not_started').mkdir()
    (root / 'state.json').write_text(json.dumps({
        'schema_version': 2, 'campaigns': [
            {'name': name, 'package': 'suave_runner',
             'config_file': f'config/runner/{name}_runner_config.yml'}
            for name in (*names, 'not_started')]}))
    return sorted(campaigns)


@pytest.mark.parametrize('relative', ['', 'campaigns', 'state.json'])
def test_batch_forms_share_one_layout(tmp_path, relative):
    """A batch, its campaigns/ dir and state.json resolve identically."""
    campaigns = _batch(tmp_path)
    layout = batch_layout(tmp_path / relative)
    assert layout.root == tmp_path
    assert layout.campaigns == campaigns
    assert layout.manifest == tmp_path / 'state.json'
    assert layout.results_root == tmp_path / 'campaign_results'
    assert layout.latex_root == tmp_path / 'campaigns_latex_tables'


def test_campaign_inside_batch_uses_batch_root(tmp_path):
    """One campaign of a batch keeps the batch's output roots."""
    campaign = _batch(tmp_path)[0]
    layout = batch_layout(campaign)
    assert layout.root == tmp_path
    assert layout.campaigns == [campaign]
    assert layout.manifest == tmp_path / 'state.json'


def test_lone_campaign_root_is_its_parent(tmp_path):
    """A campaign outside a batch writes next to itself."""
    campaign = _campaign(tmp_path / 'results' / '2026_10_11_12-00-00')
    layout = batch_layout(campaign)
    assert layout.root == tmp_path / 'results'
    assert layout.campaigns == [campaign]
    assert layout.manifest is None
    assert layout.results_root == tmp_path / 'results' / 'campaign_results'


def test_batch_without_manifest(tmp_path):
    """A campaigns/ folder alone is enough to find campaigns."""
    campaign = _campaign(tmp_path / 'campaigns' / 'exp1')
    layout = batch_layout(tmp_path)
    assert layout.campaigns == [campaign]
    assert layout.manifest is None


@pytest.mark.parametrize('manifest', [
    {'schema_version': 1, 'campaigns': {}},
    {'campaigns': []},
    {'schema_version': 2, 'campaigns': {}},
    {'schema_version': 2, 'campaigns': [{'name': 'exp1'}]},
    [],
])
def test_bad_manifest_schema(tmp_path, manifest):
    """Only the schema-2 manifest is accepted."""
    (tmp_path / 'state.json').write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='state-file schema|campaign list'):
        load_manifest(tmp_path / 'state.json')
    with pytest.raises(ValueError, match='state-file schema|campaign list'):
        batch_layout(tmp_path)


def test_unreadable_manifest(tmp_path):
    """Report a corrupt state.json instead of a JSON traceback."""
    (tmp_path / 'state.json').write_text('{')
    with pytest.raises(ValueError, match='cannot read state file'):
        load_manifest(tmp_path / 'state.json')


@pytest.mark.parametrize('name', ['missing', 'plain', 'file.txt'])
def test_not_a_batch_or_campaign(tmp_path, name):
    """Reject paths that are neither batches nor campaigns."""
    (tmp_path / 'plain').mkdir()
    (tmp_path / 'file.txt').write_text('x')
    with pytest.raises(ValueError, match='not found|expected a batch|'
                       'not a batch manifest'):
        batch_layout(tmp_path / name)
