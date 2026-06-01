"""
BFS / Wavefront exploration strategy.

Based on the Wavefront Frontier concept
"""

import math
from collections import deque
from typing import Optional
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid
from .base import BaseExplorationStrategy
from room_exploration.map_utils import (
    world_to_grid,
    grid_to_world,
    is_free,
    is_unknown,
    in_bounds,
    get_neighbors_4,
    make_goal_pose,
    has_obstacle_clearance,
)

class BFSExploration(BaseExplorationStrategy):

    def __init__(self, node, params: dict):
        super().__init__(node, params)

        # 4-connected
        self.clearance_cells = params.get('clearance_cells', 4)
        self.goal_standoff_cells = params.get('goal_standoff_cells', 0)
        self.min_frontier_size = params.get('min_frontier_size', 3)

        self.node.get_logger().info(
            f'BFS exploration initialized: '
            f'clearance_cells={self.clearance_cells}, '
            f'goal_standoff_cells={self.goal_standoff_cells}, '
            f'min_frontier_size={self.min_frontier_size}, '
            f'min_distance={self.min_goal_distance}m'
        )

    def get_next_goal(
        self, grid_map: OccupancyGrid, robot_pose: Pose
    ) -> Optional[PoseStamped]:
        goal = self._bfs_for_goal(grid_map, robot_pose)

        # Only clear blacklist once all reachable frontiers have been exhausted
        if goal is None and self.blacklist:
            self.node.get_logger().info(
                f'All reachable frontiers blacklisted — clearing {len(self.blacklist)} entries and retrying.'
            )
            self.blacklist.clear()
            goal = self._bfs_for_goal(grid_map, robot_pose)

        return goal

    def _bfs_for_goal(
        self, grid_map: OccupancyGrid, robot_pose: Pose
    ) -> Optional[PoseStamped]:
        """BFS from robot through free cells. First valid frontier encountered = goal."""
        width = grid_map.info.width
        height = grid_map.info.height
        data = grid_map.data
        map_info = grid_map.info
        rejected_clearance = 0
        rejected_size = 0

        robot_gx, robot_gy = world_to_grid(
            robot_pose.position.x, robot_pose.position.y, map_info
        )

        if not in_bounds(robot_gx, robot_gy, width, height):
            robot_gx = max(0, min(robot_gx, width - 1))
            robot_gy = max(0, min(robot_gy, height - 1))

        visited = set()
        queue = deque()
        start = (robot_gx, robot_gy)
        parents: dict[tuple[int, int], tuple[int, int] | None] = {start: None}
        visited.add(start)
        queue.append(start)

        while queue:
            gx, gy = queue.popleft()

            if is_free(data, gx, gy, width):
                is_frontier = False
                for nx, ny in get_neighbors_4(gx, gy, width, height):
                    if is_unknown(data, nx, ny, width):
                        is_frontier = True
                        break

                if is_frontier:
                    goal_gx, goal_gy = self._standoff_goal_cell(gx, gy, parents)
                    wx, wy = grid_to_world(goal_gx, goal_gy, map_info)

                    if not self.is_goal_valid(wx, wy, robot_pose):
                        pass  # too close or blacklisted — fall through to expansion
                    elif not has_obstacle_clearance(goal_gx, goal_gy, self.clearance_cells, data, width, height):
                        rejected_clearance += 1
                    elif self._frontier_cluster_size(gx, gy, data, width, height) < self.min_frontier_size:
                        rejected_size += 1
                    else:
                        dist = math.hypot(wx - robot_pose.position.x, wy - robot_pose.position.y)
                        self.node.get_logger().info(
                            f'BFS selected goal ({wx:.2f}, {wy:.2f}) from frontier cell '
                            f'({gx}, {gy}), straight-line distance {dist:.2f}m'
                        )
                        return make_goal_pose(wx, wy)

            # 4-connected expansion
            for nx, ny in get_neighbors_4(gx, gy, width, height):
                if (nx, ny) not in visited and is_free(data, nx, ny, width):
                    visited.add((nx, ny))
                    parents[(nx, ny)] = (gx, gy)
                    queue.append((nx, ny))

        self.node.get_logger().info(
            f'BFS found no reachable frontiers. '
            f'Rejected {rejected_clearance} for clearance, {rejected_size} for cluster size.'
        )
        return None

    def _standoff_goal_cell(
        self,
        frontier_gx: int,
        frontier_gy: int,
        parents: dict,
    ) -> tuple[int, int]:
        """Move the goal a few BFS steps back from the frontier into known free space."""
        cell = (frontier_gx, frontier_gy)
        for _ in range(self.goal_standoff_cells):
            parent = parents.get(cell)
            if parent is None:
                break
            cell = parent
        return cell

    def _frontier_cluster_size(
        self, start_gx: int, start_gy: int, data, width: int, height: int
    ) -> int:
        """Local 4-connected flood-fill counting frontier cells. Caps at min_frontier_size."""
        cap = self.min_frontier_size
        visited_local = {(start_gx, start_gy)}
        queue = deque([(start_gx, start_gy)])
        count = 0
        while queue and count < cap:
            cx, cy = queue.popleft()
            if not is_free(data, cx, cy, width):
                continue
            is_frontier = False
            for nx, ny in get_neighbors_4(cx, cy, width, height):
                if is_unknown(data, nx, ny, width):
                    is_frontier = True
                    break
            if not is_frontier:
                continue
            count += 1
            for nx, ny in get_neighbors_4(cx, cy, width, height):
                if (nx, ny) not in visited_local:
                    visited_local.add((nx, ny))
                    queue.append((nx, ny))
        return count

    def get_name(self) -> str:
        return 'BFS Wavefront'
