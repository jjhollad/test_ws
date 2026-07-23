# Copyright 2026 jjhollad
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

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

import yaml


EXPECTED_EVENT_COLUMNS = [
    'utc_timestamp',
    'ros_time_nanoseconds',
    'event',
    'operator',
    'scenario',
]


def dataset_split(run_id):
    """Return a stable 80/10/10 split derived from the complete run ID."""
    bucket = int(hashlib.sha256(run_id.encode()).hexdigest(), 16) % 10
    if bucket == 0:
        return 'test'
    if bucket == 1:
        return 'validation'
    return 'train'


def sha256_file(path):
    """Hash a file without loading a potentially large rosbag into memory."""
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _write_json(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    temporary.replace(path)


def validate_run(run_directory):
    """Validate a run, assign its split, summarize it, and write checksums."""
    run_directory = Path(run_directory).resolve()
    errors = []
    warnings = []
    report = {
        'run_directory': str(run_directory),
        'validated_utc': datetime.now(timezone.utc).isoformat(),
        'valid': False,
        'errors': errors,
        'warnings': warnings,
    }
    if not run_directory.is_dir():
        errors.append('Run directory does not exist.')
        return report

    metadata_path = run_directory / 'metadata.json'
    events_path = run_directory / 'events.csv'
    bag_metadata_path = run_directory / 'bag' / 'metadata.yaml'

    metadata = None
    if not metadata_path.is_file():
        errors.append('Missing metadata.json.')
    else:
        try:
            metadata = json.loads(metadata_path.read_text(encoding='utf-8'))
        except (OSError, json.JSONDecodeError) as exc:
            errors.append(f'Invalid metadata.json: {exc}')

    event_count = 0
    if not events_path.is_file():
        errors.append('Missing events.csv.')
    else:
        try:
            with events_path.open(newline='', encoding='utf-8') as stream:
                reader = csv.DictReader(stream)
                if reader.fieldnames != EXPECTED_EVENT_COLUMNS:
                    errors.append('events.csv has an unexpected header.')
                event_count = sum(1 for _row in reader)
        except OSError as exc:
            errors.append(f'Cannot read events.csv: {exc}')

    bag_summary = {}
    if not bag_metadata_path.is_file():
        errors.append('Missing bag/metadata.yaml.')
    else:
        try:
            bag_metadata = yaml.safe_load(
                bag_metadata_path.read_text(encoding='utf-8')
            )
            info = bag_metadata.get('rosbag2_bagfile_information', {})
            message_count = int(info.get('message_count', 0))
            duration = info.get('duration', {}).get('nanoseconds', 0)
            topics = info.get('topics_with_message_count', [])
            bag_summary = {
                'message_count': message_count,
                'duration_nanoseconds': int(duration),
                'topic_count': len(topics),
                'topics': [
                    {
                        'name': item.get('topic_metadata', {}).get('name'),
                        'type': item.get('topic_metadata', {}).get('type'),
                        'message_count': int(item.get('message_count', 0)),
                    }
                    for item in topics
                ],
            }
            if message_count == 0:
                errors.append('Rosbag contains no messages.')
        except (
            AttributeError, OSError, TypeError, ValueError, yaml.YAMLError,
        ) as exc:
            errors.append(f'Invalid bag metadata: {exc}')

    if not (run_directory / 'calibration').is_dir():
        warnings.append('No calibration directory was captured for this run.')

    run_id = (
        metadata.get('run_id', run_directory.name)
        if isinstance(metadata, dict) else run_directory.name
    )
    split = dataset_split(run_id)
    report.update({
        'run_id': run_id,
        'dataset_split': split,
        'event_count': event_count,
        'bag': bag_summary,
        'valid': not errors,
    })

    if isinstance(metadata, dict):
        metadata['dataset_split'] = split
        metadata['last_validation_utc'] = report['validated_utc']
        metadata['last_validation_valid'] = report['valid']
        _write_json(metadata_path, metadata)

    _write_json(run_directory / 'summary.json', report)

    checksum_lines = []
    for path in sorted(run_directory.rglob('*')):
        if path.is_file() and path.name != 'checksums.sha256':
            relative = path.relative_to(run_directory)
            checksum_lines.append(f'{sha256_file(path)}  {relative.as_posix()}')
    (run_directory / 'checksums.sha256').write_text(
        '\n'.join(checksum_lines) + '\n',
        encoding='utf-8',
    )
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description='Validate a robot run dataset.')
    parser.add_argument('run_directory')
    arguments = parser.parse_args(argv)
    report = validate_run(arguments.run_directory)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report['valid'] else 1


if __name__ == '__main__':
    sys.exit(main())
