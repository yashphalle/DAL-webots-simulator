"""Occupancy grid from LIDAR: log-odds internally, probability in self.grid [0,1]."""

import math
import numpy as np


def _logodds_to_prob(l):
    return 1.0 - 1.0 / (1.0 + np.exp(l))


class OccupancyGrid:
    L_FREE = 0.2
    L_OCC = 0.7
    L_MIN = -4.0
    L_MAX = 4.0
    L_FREEZE_FREE = -2.0
    L_FREEZE_OCC = 2.0

    def __init__(self, x_min, x_max, y_min, y_max, resolution=0.25):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.resolution = resolution

        self.width = int(math.ceil((x_max - x_min) / resolution))
        self.height = int(math.ceil((y_max - y_min) / resolution))

        self._logodds = np.zeros((self.height, self.width), dtype=np.float32)
        self._frozen = np.zeros((self.height, self.width), dtype=bool)
        self.grid = np.full((self.height, self.width), 0.5, dtype=np.float32)

    def world_to_grid(self, wx, wy):
        col = int((wx - self.x_min) / self.resolution)
        row = int((wy - self.y_min) / self.resolution)
        col = max(0, min(self.width - 1, col))
        row = max(0, min(self.height - 1, row))
        return col, row

    def grid_to_world(self, col, row):
        wx = self.x_min + (col + 0.5) * self.resolution
        wy = self.y_min + (row + 0.5) * self.resolution
        return wx, wy

    def update_from_lidar(self, robot_x, robot_y, robot_heading, ranges,
                          angle_min=0.0, angle_increment=None, max_range=3.5):
        if len(ranges) == 0:
            return

        if angle_increment is None:
            angle_increment = 2.0 * math.pi / len(ranges)

        robot_col, robot_row = self.world_to_grid(robot_x, robot_y)

        for i, r in enumerate(ranges):
            beam_angle = robot_heading + angle_min + i * angle_increment

            if r <= 0 or math.isinf(r) or math.isnan(r):
                continue

            hit_obstacle = r < max_range
            cast_range = min(r, max_range)
            end_x = robot_x + cast_range * math.cos(beam_angle)
            end_y = robot_y + cast_range * math.sin(beam_angle)
            end_col, end_row = self.world_to_grid(end_x, end_y)

            cells = self._ray_to_cells(robot_col, robot_row, end_col, end_row)
            for col, row in cells[:-1]:
                if 0 <= col < self.width and 0 <= row < self.height:
                    if self._frozen[row, col]:
                        continue
                    val = self._logodds[row, col] - self.L_FREE
                    self._logodds[row, col] = max(self.L_MIN, val)
                    if val <= self.L_FREEZE_FREE:
                        self._frozen[row, col] = True

            if hit_obstacle:
                if 0 <= end_col < self.width and 0 <= end_row < self.height:
                    if not self._frozen[end_row, end_col]:
                        val = self._logodds[end_row, end_col] + self.L_OCC
                        self._logodds[end_row, end_col] = min(self.L_MAX, val)
                        if val >= self.L_FREEZE_OCC:
                            self._frozen[end_row, end_col] = True
            else:
                if 0 <= end_col < self.width and 0 <= end_row < self.height:
                    if not self._frozen[end_row, end_col]:
                        val = self._logodds[end_row, end_col] - self.L_FREE
                        self._logodds[end_row, end_col] = max(self.L_MIN, val)
                        if val <= self.L_FREEZE_FREE:
                            self._frozen[end_row, end_col] = True

        self.grid = _logodds_to_prob(self._logodds)

    @staticmethod
    def _ray_to_cells(x0, y0, x1, y1):
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            cells.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

        return cells
