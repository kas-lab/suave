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

"""Test MCAP bag metadata validation helpers."""

import pytest
from suave_runner.bag_validation import validate_bag
import yaml


class FakeReader:
    """Reader fixture that yields a fixed number of messages."""

    def __init__(self, count):
        """Store the remaining message count."""
        self.remaining = count

    def has_next(self):
        """Return whether another fake message is available."""
        return self.remaining > 0

    def read_next(self):
        """Consume one fake message."""
        self.remaining -= 1
        return None


def make_bag(tmp_path, count=2, duration=10, storage='mcap'):
    """Create a minimal metadata/storage fixture."""
    bag = tmp_path / 'bag'
    bag.mkdir()
    (bag / 'bag_0.mcap').write_bytes(b'data')
    data = {
        'rosbag2_bagfile_information': {
            'storage_identifier': storage,
            'duration': {'nanoseconds': duration},
            'message_count': count,
            'relative_file_paths': ['bag_0.mcap'],
        }
    }
    (bag / 'metadata.yaml').write_text(yaml.safe_dump(data))
    return bag


def fake_factory(count):
    """Return a fake reader factory for validation tests."""
    return lambda _path: FakeReader(count)


def test_validate_bag_accepts_matching_metadata_and_reader(tmp_path):
    """Verify valid metadata and stream count are accepted."""
    bag = make_bag(tmp_path, count=2)
    info = validate_bag(bag, reader_factory=fake_factory(2))
    assert info['message_count'] == 2


@pytest.mark.parametrize('field,value', [
    ('storage_identifier', 'sqlite3'),
    ('duration', {'nanoseconds': 0}),
    ('message_count', 0),
])
def test_validate_bag_rejects_invalid_metadata(tmp_path, field, value):
    """Verify invalid metadata is rejected before reading."""
    bag = make_bag(tmp_path)
    data = yaml.safe_load((bag / 'metadata.yaml').read_text())
    data['rosbag2_bagfile_information'][field] = value
    (bag / 'metadata.yaml').write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        validate_bag(bag, reader_factory=fake_factory(2))


def test_validate_bag_rejects_missing_storage_file(tmp_path):
    """Verify missing storage files are rejected."""
    bag = make_bag(tmp_path)
    (bag / 'bag_0.mcap').unlink()
    with pytest.raises(ValueError, match='Missing/empty'):
        validate_bag(bag, reader_factory=fake_factory(2))


def test_validate_bag_rejects_count_mismatch(tmp_path):
    """Verify stream count mismatches are rejected."""
    bag = make_bag(tmp_path, count=2)
    with pytest.raises(ValueError, match='Message count mismatch'):
        validate_bag(bag, reader_factory=fake_factory(1))


def test_validate_bag_rejects_escaping_relative_path(tmp_path):
    """Verify storage paths cannot escape the bag directory."""
    bag = make_bag(tmp_path)
    data = yaml.safe_load((bag / 'metadata.yaml').read_text())
    data['rosbag2_bagfile_information']['relative_file_paths'] = ['../x.mcap']
    (bag / 'metadata.yaml').write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='escapes'):
        validate_bag(bag, reader_factory=fake_factory(2))
