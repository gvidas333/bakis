"""Information Gain exploration strategy."""

import math
from typing import Optional

from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid

from .base import BaseExplorationStrategy
from room_exploration.map_utils import (
    world_to_grid,
    detect_frontiers,
    frontier_centroid,
    observable_unknown_count,
    make_goal_pose,
    has_obstacle_clearance,
)


class InformationGainExploration(BaseExplorationStrategy):
    def __init__(self, node, params: dict):
        super().__init__(node, params)
        self.sensor_range = params.get('sensor_range', 3.5)
        self.min_frontier_size = params.get('min_frontier_size', 3)
        self.alpha = params.get('alpha', 0.5)
        self.clearance_cells = params.get('clearance_cells', 5)

        self.node.get_logger().info(
            f'Information Gain exploration initialized: '
            f'sensor_range={self.sensor_range}m, '
            f'alpha={self.alpha}'
        )

    def get_next_goal(
        self, grid_map: OccupancyGrid, robot_pose: Pose
    ) -> Optional[PoseStamped]:
        width = grid_map.info.width
        height = grid_map.info.height
        data = grid_map.data
        map_info = grid_map.info

        resolution = map_info.resolution

        robot_x = robot_pose.position.x
        robot_y = robot_pose.position.y

        frontiers = detect_frontiers(grid_map, self.min_frontier_size)
        if not frontiers:
            self.node.get_logger().info('Information Gain: no frontiers found.')
            return None

        radius_cells = int(self.sensor_range / resolution)

        # Score every frontier 
        scored = []
        for cells in frontiers:
            cx, cy = frontier_centroid(cells, map_info, data, width, height)

            if math.hypot(cx - robot_x, cy - robot_y) < self.min_goal_distance:
                continue

            gx, gy = world_to_grid(cx, cy, map_info)

            if not has_obstacle_clearance(gx, gy, self.clearance_cells, data, width, height):
                continue

            info_gain = observable_unknown_count(
                gx, gy, radius_cells, data, width, height
            )
            dist = math.hypot(cx - robot_x, cy - robot_y)

            scored.append((info_gain, dist, cx, cy, len(cells)))

        if not scored:
            return None

        # Apply blacklist filter. If it removes everything, clear and use all
        valid = [
            s for s in scored
            if not any(math.hypot(s[2] - bx, s[3] - by) < self.blacklist_radius
                       for bx, by in self.blacklist)
        ]

        if not valid and self.blacklist:
            self.node.get_logger().info(
                f'All info-gain frontiers blacklisted — clearing {len(self.blacklist)} entries.'
            )
            self.blacklist.clear()
            valid = scored

        if not valid:
            return None

        # Normalize gain and distance to [0, 1] before combining
        max_gain = max(s[0] for s in valid)
        max_dist = max(s[1] for s in valid)

        if max_gain == 0:
            max_gain = 1
        if max_dist == 0:
            max_dist = 1.0

        # alpha=1: pure info gain. alpha=0: pure nearest. 0.5: balanced
        best_score = -1.0
        best = valid[0]
        for info_gain, dist, cx, cy, size in valid:
            norm_gain = info_gain / max_gain
            norm_inv_cost = 1.0 - (dist / max_dist)
            score = self.alpha * norm_gain + (1.0 - self.alpha) * norm_inv_cost
            if score > best_score:
                best_score = score
                best = (info_gain, dist, cx, cy, size)

        _, dist, wx, wy, size = best
        self.node.get_logger().info(
            f'InfoGain selected ({wx:.2f}, {wy:.2f}), '
            f'gain={best[0]}, dist={dist:.2f}m, size={size}'
        )

        return make_goal_pose(wx, wy)

    def get_name(self) -> str:
        return 'Information Gain'
