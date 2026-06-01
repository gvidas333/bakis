"""
Frontier-based exploration strategy

Classic frontier exploration: pick the nearest frontier centroid by Euclidean distance.
Blacklisted goals are retried only after all non-blacklisted options are exhausted.
"""

import math
from typing import Optional
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid
from .base import BaseExplorationStrategy
from room_exploration.map_utils import (
    world_to_grid,
    detect_frontiers,
    frontier_centroid,
    has_obstacle_clearance,
    make_goal_pose,
)


class FrontierExploration(BaseExplorationStrategy):

    def __init__(self, node, params: dict):
        super().__init__(node, params)
        self.min_goal_distance = params.get('min_goal_distance', 0.2)
        self.min_frontier_size = params.get('min_frontier_size', 5)
        self.clearance_cells = params.get('clearance_cells', 5)

        self.node.get_logger().info(
            f'Frontier exploration initialized: '
            f'min_frontier_size={self.min_frontier_size}, '
            f'clearance_cells={self.clearance_cells}'
        )

    def get_next_goal(
        self, grid_map: OccupancyGrid, robot_pose: Pose
    ) -> Optional[PoseStamped]:
        width = grid_map.info.width
        height = grid_map.info.height
        data = grid_map.data
        map_info = grid_map.info

        frontiers = detect_frontiers(grid_map, self.min_frontier_size)
        if not frontiers:
            self.node.get_logger().info('No frontiers found.')
            return None

        goal = self._pick_nearest(frontiers, robot_pose, map_info, data, width, height)

        # Only clear blacklist once all non-blacklisted options are exhausted
        if goal is None and self.blacklist:
            self.node.get_logger().info(
                f'All reachable frontiers blacklisted — clearing {len(self.blacklist)} entries and retrying.'
            )
            self.blacklist.clear()
            goal = self._pick_nearest(frontiers, robot_pose, map_info, data, width, height)

        return goal

    def _pick_nearest(
        self, frontiers, robot_pose: Pose, map_info, data, width: int, height: int
    ) -> Optional[PoseStamped]:
        robot_x = robot_pose.position.x
        robot_y = robot_pose.position.y

        best_dist = float('inf')
        best_wx = best_wy = None

        for cells in frontiers:
            wx, wy = frontier_centroid(cells, map_info, data, width, height)

            if math.hypot(wx - robot_x, wy - robot_y) < self.min_goal_distance:
                continue

            gx, gy = world_to_grid(wx, wy, map_info)
            if not has_obstacle_clearance(gx, gy, self.clearance_cells, data, width, height):
                continue

            if any(math.hypot(wx - bx, wy - by) < self.blacklist_radius for bx, by in self.blacklist):
                continue

            dist = math.hypot(wx - robot_x, wy - robot_y)
            if dist < best_dist:
                best_dist = dist
                best_wx, best_wy = wx, wy

        if best_wx is None or best_wy is None:
            return None

        self.node.get_logger().info(
            f'Frontier selected at ({best_wx:.2f}, {best_wy:.2f}), dist={best_dist:.2f}m'
        )
        return make_goal_pose(best_wx, best_wy)

    def get_name(self) -> str:
        return 'Frontier Exploration'
