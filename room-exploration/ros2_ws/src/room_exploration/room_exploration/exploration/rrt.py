"""
RRT-based exploration strategy
"""

from math import hypot
from random import random, uniform
from typing import List, Optional, Tuple
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid
from .base import BaseExplorationStrategy
from room_exploration.map_utils import (
    world_to_grid,
    is_free,
    is_unknown,
    in_bounds,
    get_neighbors_4,
    observable_unknown_count,
    make_goal_pose,
    has_obstacle_clearance,
)


class RRTExploration(BaseExplorationStrategy):

    def __init__(self, node, params: dict):
        super().__init__(node, params)

        self.step_size = params.get('step_size', 0.5)
        self.max_iterations = params.get('max_iterations', 250)
        self.goal_bias = params.get('goal_bias', 0.1)
        self.sensor_range = params.get('sensor_range', 3.5)
        self.cost_weight = params.get('cost_weight', 0.5)
        self.clearance_cells = params.get('clearance_cells', 5)

        self.node.get_logger().info(
            f'RRT exploration initialized: '
            f'step={self.step_size}m, '
            f'max_iter={self.max_iterations}, '
            f'bias={self.goal_bias}'
        )

    def get_next_goal(
        self, grid_map: OccupancyGrid, robot_pose: Pose
    ) -> Optional[PoseStamped]:
        return self._rrt_for_goal(grid_map, robot_pose)

    def _rrt_for_goal(
        self, grid_map: OccupancyGrid, robot_pose: Pose
    ) -> Optional[PoseStamped]:
        width = grid_map.info.width
        height = grid_map.info.height
        data = grid_map.data
        map_info = grid_map.info
        resolution = map_info.resolution

        origin_x = map_info.origin.position.x
        origin_y = map_info.origin.position.y

        robot_x = robot_pose.position.x
        robot_y = robot_pose.position.y

        map_min_x = origin_x
        map_min_y = origin_y
        map_max_x = origin_x + width * resolution
        map_max_y = origin_y + height * resolution

        tree: List[Tuple[float, float]] = [(robot_x, robot_y)]
        parents: List[int] = [-1]    # root has no parent
        costs: List[float] = [0.0]   # root has zero cost

        frontier_nodes: List[Tuple[float, float, float]] = []

        radius_cells = int(self.sensor_range / resolution)

        for _ in range(self.max_iterations):
            if random() < self.goal_bias:
                sample = self._sample_unknown(
                    data, width, height, map_info,
                    map_min_x, map_min_y, map_max_x, map_max_y
                )
                if sample is None:
                    sample = self._sample_random(map_min_x, map_min_y, map_max_x, map_max_y)
            else:
                sample = self._sample_random(map_min_x, map_min_y, map_max_x, map_max_y)

            sx, sy = sample

            nearest_idx = min(
                range(len(tree)),
                key=lambda i: (tree[i][0] - sx) ** 2 + (tree[i][1] - sy) ** 2
            )
            nx, ny = tree[nearest_idx]

            dx = sx - nx
            dy = sy - ny
            dist = hypot(dx, dy)
            if dist < 1e-6:
                continue

            edge_dist = min(self.step_size, dist)
            new_x = nx + (dx / dist) * edge_dist
            new_y = ny + (dy / dist) * edge_dist

            gx, gy = world_to_grid(new_x, new_y, map_info)
            if not in_bounds(gx, gy, width, height):
                continue
            gx_near, gy_near = world_to_grid(nx, ny, map_info)
            if not self._path_clear(gx_near, gy_near, gx, gy, data, width, height):
                continue

            new_cost = costs[nearest_idx] + edge_dist
            tree.append((new_x, new_y))
            parents.append(nearest_idx)
            costs.append(new_cost)

            is_frontier = False
            for nbx, nby in get_neighbors_4(gx, gy, width, height):
                if is_unknown(data, nbx, nby, width):
                    is_frontier = True
                    break

            if (is_frontier
                    and has_obstacle_clearance(gx, gy, self.clearance_cells, data, width, height)
                    and hypot(new_x - robot_x, new_y - robot_y) >= self.min_goal_distance):
                frontier_nodes.append((new_x, new_y, new_cost))

        if not frontier_nodes:
            self.node.get_logger().info('RRT found no frontier nodes.')
            return None

        # Apply blacklist filter. If it removes everything, clear and use all
        valid_nodes = [
            node for node in frontier_nodes
            if not any(
                hypot(node[0] - bx, node[1] - by) < self.blacklist_radius
                for bx, by in self.blacklist
            )
        ]

        if not valid_nodes and self.blacklist:
            self.node.get_logger().info(
                f'All RRT frontiers blacklisted — clearing {len(self.blacklist)} entries.'
            )
            self.blacklist.clear()
            valid_nodes = frontier_nodes

        if not valid_nodes:
            return None

        # Score: info gain (cells) minus tree-path cost (cells), kept in consistent units
        best_score = -float('inf')
        best_node = valid_nodes[0]

        for fx, fy, tree_cost in valid_nodes:
            gx, gy = world_to_grid(fx, fy, map_info)
            info = observable_unknown_count(gx, gy, radius_cells, data, width, height)
            cost_cells = tree_cost / resolution
            score = info - self.cost_weight * cost_cells
            if score > best_score:
                best_score = score
                best_node = (fx, fy, tree_cost)

        wx, wy, _ = best_node
        self.node.get_logger().info(
            f'RRT selected ({wx:.2f}, {wy:.2f}), '
            f'tree_size={len(tree)}, '
            f'frontiers_found={len(frontier_nodes)}'
        )
        return make_goal_pose(wx, wy)

    def _sample_random(
        self, min_x: float, min_y: float, max_x: float, max_y: float
    ) -> Tuple[float, float]:
        return uniform(min_x, max_x), uniform(min_y, max_y)

    def _sample_unknown(
        self, data, width: int, height: int, map_info,
        min_x: float, min_y: float, max_x: float, max_y: float
    ) -> Optional[Tuple[float, float]]:
        for _ in range(10):
            wx = uniform(min_x, max_x)
            wy = uniform(min_y, max_y)
            gx, gy = world_to_grid(wx, wy, map_info)
            if in_bounds(gx, gy, width, height) and is_unknown(data, gx, gy, width):
                return wx, wy
        return None

    def _path_clear(
        self,
        gx0: int, gy0: int,
        gx1: int, gy1: int,
        grid_data, width: int, height: int,
    ) -> bool:
        """Bresenham line walk — True only if every cell is known and free."""
        dx = abs(gx1 - gx0)
        dy = abs(gy1 - gy0)
        sx = 1 if gx0 < gx1 else -1
        sy = 1 if gy0 < gy1 else -1
        err = dx - dy
        cx, cy = gx0, gy0

        while True:
            if not in_bounds(cx, cy, width, height):
                return False
            if not is_free(grid_data, cx, cy, width):
                return False
            if cx == gx1 and cy == gy1:
                return True
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                cx += sx
            if e2 < dx:
                err += dx
                cy += sy

    def get_name(self) -> str:
        return 'RRT'
