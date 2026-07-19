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

from nav_msgs.msg import OccupancyGrid

from robot_run_manager.frontier_mapper import frontier_cells


def test_frontier_cells_find_free_unknown_boundary():
    grid = OccupancyGrid()
    grid.info.width = 7
    grid.info.height = 7
    grid.data = [-1] * 49
    for row in range(2, 5):
        for col in range(2, 5):
            grid.data[row * 7 + col] = 0

    cells = frontier_cells(grid, clearance_cells=0)

    assert (3, 3) not in cells
    assert (2, 2) in cells


def test_frontier_cells_respect_occupied_clearance():
    grid = OccupancyGrid()
    grid.info.width = 7
    grid.info.height = 7
    grid.data = [-1] * 49
    grid.data[3 * 7 + 3] = 0
    grid.data[3 * 7 + 4] = 100

    assert frontier_cells(grid, clearance_cells=1) == []
