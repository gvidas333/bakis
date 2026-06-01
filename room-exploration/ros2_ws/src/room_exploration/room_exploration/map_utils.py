"""
Shared occupancy grid utilities used by all exploration algorithms.

Occupancy grid values (from SLAM Toolbox):
  -1   = unknown (not yet observed)
   0   = free (definitely empty)
  100  = occupied (wall/obstacle)
  1-99 = probability of occupancy
"""

from collections import deque
from typing import List, Tuple

import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid


FREE_THRESHOLD = 50

_DIRS_4 = [(1, 0), (-1, 0), (0, 1), (0, -1)]
_DIRS_8 = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]


def world_to_grid(wx: float, wy: float, map_info) -> Tuple[int, int]:
    resolution = map_info.resolution
    origin = map_info.origin.position
    gx = int((wx - origin.x) / resolution)
    gy = int((wy - origin.y) / resolution)
    return gx, gy


def grid_to_world(gx: int, gy: int, map_info) -> Tuple[float, float]:
    resolution = map_info.resolution
    origin = map_info.origin.position
    # +0.5 shifts from cell corner to cell center. Without it, centroids
    # would land at the bottom-left of the cell instead of the middle
    wx = origin.x + (gx + 0.5) * resolution
    wy = origin.y + (gy + 0.5) * resolution
    return wx, wy


def grid_index(gx: int, gy: int, width: int) -> int:
    return gy * width + gx


def in_bounds(gx: int, gy: int, width: int, height: int) -> bool:
    return 0 <= gx < width and 0 <= gy < height


def is_free(grid_data, gx: int, gy: int, width: int) -> bool:
    val = grid_data[grid_index(gx, gy, width)]
    return 0 <= val < FREE_THRESHOLD


def is_occupied(grid_data, gx: int, gy: int, width: int) -> bool:
    val = grid_data[grid_index(gx, gy, width)]
    return val >= FREE_THRESHOLD


def is_unknown(grid_data, gx: int, gy: int, width: int) -> bool:
    return grid_data[grid_index(gx, gy, width)] == -1


def get_neighbors_4(gx: int, gy: int, width: int, height: int) -> List[Tuple[int, int]]:
    return [
        (gx + dx, gy + dy)
        for dx, dy in _DIRS_4
        if in_bounds(gx + dx, gy + dy, width, height)
    ]


def get_neighbors_8(gx: int, gy: int, width: int, height: int) -> List[Tuple[int, int]]:
    return [
        (gx + dx, gy + dy)
        for dx, dy in _DIRS_8
        if in_bounds(gx + dx, gy + dy, width, height)
    ]


def _dilate_mask(mask: np.ndarray, iterations: int = 1) -> np.ndarray:
    """4-connected binary dilation: grow True regions outward by `iterations` cells."""
    result = mask.copy()
    for _ in range(iterations):
        shifted = result.copy()
        shifted[1:, :]  |= result[:-1, :]
        shifted[:-1, :] |= result[1:, :]
        shifted[:, 1:]  |= result[:, :-1]
        shifted[:, :-1] |= result[:, 1:]
        result = shifted
    return result


def detect_frontiers(
    grid_map: OccupancyGrid, min_size: int = 3
) -> List[List[Tuple[int, int]]]:
    width = grid_map.info.width
    height = grid_map.info.height

    grid_2d = np.array(grid_map.data, dtype=np.int8).reshape((height, width))
    free_mask = (grid_2d >= 0) & (grid_2d < FREE_THRESHOLD)
    unknown_mask = (grid_2d == -1)

    frontier_mask = free_mask & _dilate_mask(unknown_mask, iterations=1)

    if not frontier_mask.any():
        return []

    visited = np.zeros_like(frontier_mask)
    clusters: List[List[Tuple[int, int]]] = []

    for start_r, start_c in zip(*np.where(frontier_mask)):
        start_r, start_c = int(start_r), int(start_c)
        if visited[start_r, start_c]:
            continue

        cluster: List[Tuple[int, int]] = []
        queue = deque([(start_r, start_c)])
        visited[start_r, start_c] = True

        while queue:
            r, c = queue.popleft()
            cluster.append((c, r))  # convert (row, col) → (gx, gy)
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    if dr == 0 and dc == 0:
                        continue
                    nr, nc = r + dr, c + dc
                    if (0 <= nr < height and 0 <= nc < width
                            and frontier_mask[nr, nc]
                            and not visited[nr, nc]):
                        visited[nr, nc] = True
                        queue.append((nr, nc))

        if len(cluster) >= min_size:
            clusters.append(cluster)

    return clusters


def frontier_centroid(
    cells: List[Tuple[int, int]],
    map_info,
    grid_data,
    width: int,
    height: int,
) -> Tuple[float, float]:
    """Return the world-coordinate centroid of a frontier cluster

    If the average position lands in a wall or out of bounds, falls back
    to the nearest actual cluster cell instead
    """
    avg_gx = int(sum(c[0] for c in cells) / len(cells))
    avg_gy = int(sum(c[1] for c in cells) / len(cells))
    wx, wy = grid_to_world(avg_gx, avg_gy, map_info)

    if in_bounds(avg_gx, avg_gy, width, height) and is_free(grid_data, avg_gx, avg_gy, width):
        return wx, wy

    nearest = min(cells, key=lambda c: (c[0] - avg_gx) ** 2 + (c[1] - avg_gy) ** 2)
    return grid_to_world(nearest[0], nearest[1], map_info)


def compute_coverage(grid_map: OccupancyGrid) -> float:
    """Ratio of known cells inside the mapped room envelope
    """
    width = grid_map.info.width
    height = grid_map.info.height

    if width == 0 or height == 0:
        return 0.0

    grid_2d = np.array(grid_map.data, dtype=np.int16).reshape((height, width))

    known_mask = grid_2d != -1
    if not known_mask.any():
        return 0.0

    unknown_mask = grid_2d == -1
    exterior_unknown = np.zeros((height, width), dtype=bool)
    queue = deque()

    for col in range(width):
        if unknown_mask[0, col]:
            exterior_unknown[0, col] = True
            queue.append((0, col))
        if unknown_mask[height - 1, col] and not exterior_unknown[height - 1, col]:
            exterior_unknown[height - 1, col] = True
            queue.append((height - 1, col))

    for row in range(height):
        if unknown_mask[row, 0] and not exterior_unknown[row, 0]:
            exterior_unknown[row, 0] = True
            queue.append((row, 0))
        if unknown_mask[row, width - 1] and not exterior_unknown[row, width - 1]:
            exterior_unknown[row, width - 1] = True
            queue.append((row, width - 1))

    while queue:
        row, col = queue.popleft()
        for d_row, d_col in _DIRS_4:
            next_row = row + d_row
            next_col = col + d_col
            if not in_bounds(next_col, next_row, width, height):
                continue
            if unknown_mask[next_row, next_col] and not exterior_unknown[next_row, next_col]:
                exterior_unknown[next_row, next_col] = True
                queue.append((next_row, next_col))

    room_envelope = ~exterior_unknown
    total_room_cells = int(room_envelope.sum())
    if total_room_cells == 0:
        return 0.0

    known_room_cells = int((known_mask & room_envelope).sum())
    return known_room_cells / total_room_cells


def observable_unknown_count(
    gx: int,
    gy: int,
    radius_cells: int,
    grid_data,
    width: int,
    height: int,
) -> int:
    """Count unknown cells reachable from (gx, gy) within radius_cells via BFS"""
    if not (0 <= gx < width and 0 <= gy < height):
        return 0

    visited = {(gx, gy)}
    queue = deque([(gx, gy)])
    unknown_count = 0

    while queue:
        cx, cy = queue.popleft()
        val = grid_data[cy * width + cx]
        if val < 0:
            unknown_count += 1
        for ddx, ddy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = cx + ddx, cy + ddy
            if (nx, ny) in visited:
                continue
            if abs(nx - gx) > radius_cells or abs(ny - gy) > radius_cells:
                continue
            if not (0 <= nx < width and 0 <= ny < height):
                continue
            if grid_data[ny * width + nx] >= FREE_THRESHOLD:
                continue
            visited.add((nx, ny))
            queue.append((nx, ny))

    return unknown_count


def has_obstacle_clearance(
    gx: int,
    gy: int,
    clearance_cells: int,
    grid_data,
    width: int,
    height: int,
) -> bool:
    """
    Reject goals where any wall lies within a Euclidean disk of clearance_cells
    """
    k = clearance_cells
    k_sq = k * k
    for dy in range(-k, k + 1):
        for dx in range(-k, k + 1):
            if dx * dx + dy * dy > k_sq:
                continue
            nx, ny = gx + dx, gy + dy
            if in_bounds(nx, ny, width, height) and is_occupied(grid_data, nx, ny, width):
                return False
    return True


def make_goal_pose(wx: float, wy: float, frame_id: str = 'map') -> PoseStamped:
    goal = PoseStamped()
    goal.header.frame_id = frame_id
    goal.pose.position.x = wx
    goal.pose.position.y = wy
    goal.pose.position.z = 0.0
    goal.pose.orientation.w = 1.0
    return goal
