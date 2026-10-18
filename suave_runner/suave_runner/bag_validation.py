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

"""Validate finalized MCAP recordings without requiring particular topics."""

from pathlib import Path

from suave_tools.runs import artifact_path

import yaml  # noqa: I201


def validate_bag(path, reader_factory=None):
    """Require sound metadata, storage files and a complete readable stream."""
    path = Path(path)
    info = yaml.safe_load((path / 'metadata.yaml').read_text())[
        'rosbag2_bagfile_information']
    if not _metadata_is_valid(info):
        raise ValueError(f'Invalid recording metadata: {path}')
    _validate_storage_files(path, info['relative_file_paths'])
    count = _stream_message_count(path, reader_factory)
    if count != info['message_count']:
        raise ValueError(f'Message count mismatch in {path}: {count}')
    return info


def _metadata_is_valid(info):
    return (info['storage_identifier'] == 'mcap' and
            info['duration']['nanoseconds'] > 0 and
            info['message_count'] > 0 and
            bool(info['relative_file_paths']))


def _validate_storage_files(path, files):
    for relative in files:
        storage = artifact_path(path, relative)
        if not storage.is_file() or storage.stat().st_size == 0:
            raise ValueError(f'Missing/empty storage file: {storage}')


def _stream_message_count(path, reader_factory):
    if reader_factory is None:
        from rosbag2_py import ConverterOptions
        from rosbag2_py import SequentialReader
        from rosbag2_py import StorageOptions
        reader = SequentialReader()
        reader.open(StorageOptions(uri=str(path), storage_id='mcap'),
                    ConverterOptions('', ''))
    else:
        reader = reader_factory(path)
    count = 0
    while reader.has_next():
        reader.read_next()
        count += 1
    del reader
    return count
