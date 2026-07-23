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

import csv
import json

from robot_run_manager.dataset import dataset_split, validate_run


def test_dataset_split_is_stable():
    assert dataset_split('run-001') == dataset_split('run-001')
    assert dataset_split('run-001') in {'train', 'validation', 'test'}


def test_validate_run(tmp_path):
    run = tmp_path / 'run-001'
    bag = run / 'bag'
    bag.mkdir(parents=True)
    (run / 'metadata.json').write_text(
        json.dumps({'run_id': 'run-001', 'status': 'complete'}),
        encoding='utf-8',
    )
    with (run / 'events.csv').open('w', newline='', encoding='utf-8') as stream:
        csv.writer(stream).writerow([
            'utc_timestamp',
            'ros_time_nanoseconds',
            'event',
            'operator',
            'scenario',
        ])
    (bag / 'metadata.yaml').write_text(
        'rosbag2_bagfile_information:\n'
        '  duration: {nanoseconds: 10}\n'
        '  message_count: 1\n'
        '  topics_with_message_count: []\n',
        encoding='utf-8',
    )
    (bag / 'bag_0.db3').write_bytes(b'test bag bytes')

    report = validate_run(run)

    assert report['valid']
    assert (run / 'summary.json').is_file()
    assert (run / 'checksums.sha256').is_file()
    assert 'bag/bag_0.db3' in (run / 'checksums.sha256').read_text()
