"""
Voronoi/GVD-based exploration strategy
"""

from heapq import heappop, heappush
import math
from typing import List, Optional, Tuple
import numpy as np
from scipy.ndimage import maximum_filter, distance_transform_edt
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid
from .base import BaseExplorationStrategy
from room_exploration.map_utils import (
    world_to_grid,
    grid_to_world,
    detect_frontiers,
    frontier_centroid,
    observable_unknown_count,
    has_obstacle_clearance,
    make_goal_pose,
    FREE_THRESHOLD,
)


class VoronoiExploration(BaseExplorationStrategy):

    def __init__(self, node, params: dict):
        super().__init__(node, params)

        self.sensor_range = params.get('sensor_range', 3.5)
        self.cost_weight = params.get('cost_weight', 0.5)
        self.min_ridge_dist = params.get('min_ridge_dist', 3)
        self.min_frontier_size = params.get('min_frontier_size', 3)
        self.clearance_cells = params.get('clearance_cells', 5)

        self.node.get_logger().info(
            f'Voronoi exploration initialized: '
            f'sensor_range={self.sensor_range}m, '
            f'cost_weight={self.cost_weight}, '
            f'min_ridge_dist={self.min_ridge_dist} cells, '
            f'clearance_cells={self.clearance_cells}'
        )

    #Override to satisfy base class
    def get_next_goal(
        self, grid_map: OccupancyGrid, robot_pose: Pose
    ) -> Optional[PoseStamped]:
        poses = self.get_next_path(grid_map, robot_pose)
        return poses[-1] if poses else None

    def get_next_path(
        self, grid_map: OccupancyGrid, robot_pose: Pose
    ) -> Optional[List[PoseStamped]]:
        sol = self._compute_solution(grid_map, robot_pose)
        if sol is None:
            return None
        ridge_path, (wx, wy) = sol
        map_info = grid_map.info

        # ~30cm waypoint spacing
        step = max(1, int(round(0.30 / map_info.resolution)))

        poses: List[PoseStamped] = []
        if len(ridge_path) > 1:
            indices = list(range(step, len(ridge_path), step))
            if not indices or indices[-1] != len(ridge_path) - 1:
                indices.append(len(ridge_path) - 1)
            for i in indices:
                r, c = ridge_path[i]
                rwx, rwy = grid_to_world(c, r, map_info)
                if math.hypot(rwx - robot_pose.position.x, rwy - robot_pose.position.y) >= self.min_goal_distance:
                    poses.append(make_goal_pose(rwx, rwy))

        if math.hypot(wx - robot_pose.position.x, wy - robot_pose.position.y) >= self.min_goal_distance:
            poses.append(make_goal_pose(wx, wy))

        if not poses:
            self.node.get_logger().info('Voronoi: selected path made no meaningful progress; skipping goal.')
            return None

        return poses

    def _compute_solution(
        self, grid_map: OccupancyGrid, robot_pose: Pose
    ) -> Optional[Tuple[List[Tuple[int, int]], Tuple[float, float]]]:
        width = grid_map.info.width
        height = grid_map.info.height
        data = grid_map.data
        map_info = grid_map.info
        resolution = map_info.resolution

        robot_x = robot_pose.position.x
        robot_y = robot_pose.position.y

        grid_2d = np.array(data, dtype=np.int8).reshape((height, width))
        free_mask = (grid_2d >= 0) & (grid_2d < FREE_THRESHOLD)

        dist_transform = distance_transform_edt(free_mask)

        ridge_mask = self._extract_ridges(dist_transform, free_mask)

        robot_gx, robot_gy = world_to_grid(robot_x, robot_y, map_info)
        robot_ridge = self._find_nearest_ridge_cell(robot_gy, robot_gx, ridge_mask)
        if robot_ridge is None:
            self.node.get_logger().info(
                f'Voronoi: ridge mask is empty (no free cell >= {self.min_ridge_dist} cells from a wall). '
                f'Lower min_ridge_dist if this persists.'
            )
            return None

        graph = self._build_ridge_graph(ridge_mask)
        distances, predecessors = self._dijkstra_with_paths(robot_ridge, graph)

        frontiers = detect_frontiers(grid_map, min_size=self.min_frontier_size)
        if not frontiers:
            self.node.get_logger().info('Voronoi: no frontiers found.')
            return None

        radius_cells = int(self.sensor_range / resolution)

        scored = []
        n_invalid = n_clearance = n_disconnected = 0
        for cells in frontiers:
            cx, cy = frontier_centroid(cells, map_info, data, width, height)

            if not self.is_goal_valid(cx, cy, robot_pose):
                n_invalid += 1
                continue

            gx, gy = world_to_grid(cx, cy, map_info)

            if not has_obstacle_clearance(gx, gy, self.clearance_cells, data, width, height):
                n_clearance += 1
                continue

            frontier_ridge = self._find_nearest_ridge_cell(gy, gx, ridge_mask)
            if frontier_ridge is None or frontier_ridge not in distances:
                nearest_reachable = self._find_nearest_reachable_ridge_cell(gy, gx, distances)
                if nearest_reachable is not None:
                    nr, nc = nearest_reachable
                    bridge = math.hypot((gy - nr) * resolution, (gx - nc) * resolution)
                    gvd_path_dist = distances[nearest_reachable] * resolution + bridge
                    frontier_ridge_for_path = nearest_reachable
                else:
                    gvd_path_dist = math.hypot(cx - robot_x, cy - robot_y)
                    frontier_ridge_for_path = robot_ridge
                n_disconnected += 1
            else:
                gvd_path_dist = distances[frontier_ridge] * resolution
                frontier_ridge_for_path = frontier_ridge

            endpoint_row, endpoint_col = frontier_ridge_for_path
            endpoint_wx, endpoint_wy = grid_to_world(endpoint_col, endpoint_row, map_info)
            endpoint_dist = math.hypot(endpoint_wx - robot_x, endpoint_wy - robot_y)
            centroid_dist = math.hypot(cx - robot_x, cy - robot_y)
            if endpoint_dist < self.min_goal_distance and centroid_dist < self.min_goal_distance:
                n_invalid += 1
                continue

            info_gain = observable_unknown_count(
                gx, gy, radius_cells, data, width, height
            )

            # U_f = I_f - h * N_f 
            gvd_path_cells = gvd_path_dist / resolution
            utility = info_gain - self.cost_weight * gvd_path_cells

            scored.append((utility, cx, cy, info_gain, gvd_path_dist, len(cells), frontier_ridge_for_path))

        if not scored:
            self.node.get_logger().warn(
                f'Voronoi: all {len(frontiers)} frontiers filtered — '
                f'invalid/close={n_invalid}, no_clearance={n_clearance}'
            )
            return None

        if n_disconnected:
            self.node.get_logger().info(
                f'Voronoi: {n_disconnected}/{len(scored)} frontiers used nearest-reachable fallback '
                f'(ridge graph disconnected)'
            )

        scored.sort(key=lambda s: s[0], reverse=True)
        utility, wx, wy, info, dist, size, frontier_ridge = scored[0]

        self.node.get_logger().info(
            f'Voronoi selected ({wx:.2f}, {wy:.2f}), '
            f'utility={utility:.1f}, info={info}, '
            f'gvd_dist={dist:.2f}m, size={size}'
        )

        ridge_path = self._reconstruct_path(frontier_ridge, predecessors)
        return ridge_path, (wx, wy)

    def _extract_ridges(
        self, dist_transform: np.ndarray, free_mask: np.ndarray
    ) -> np.ndarray:
        """
        A cell is a ridge point if:
        1. It's in free space
        2. Its distance value is within 1.0 of the local 3x3 maximum
           (relaxed from strict equality so ridges form connected strips
            along curved corridors instead of isolated peaks)
        3. Its distance from obstacles is at least min_ridge_dist
        """
        local_max = maximum_filter(dist_transform, size=3)

        ridge_mask = (
            free_mask
            & (dist_transform >= local_max - 1.0)
            & (dist_transform >= self.min_ridge_dist)
        )
        return ridge_mask

    def _build_ridge_graph(self, ridge_mask: np.ndarray) -> dict:
        """Build adjacency list of the GVD ridge cells.

        Each ridge cell is a node; 8-connected ridge neighbors are edges
        with weight 1 (orthogonal) or sqrt(2) (diagonal). The resulting
        graph is the topological skeleton over which Dijkstra runs.
        """
        rows, cols = np.where(ridge_mask)
        ridge_set = set(zip(rows.tolist(), cols.tolist()))
        graph: dict = {}
        sqrt2 = math.sqrt(2.0)
        offsets = (
            (-1, -1, sqrt2), (-1, 0, 1.0), (-1, 1, sqrt2),
            (0, -1, 1.0),                  (0, 1, 1.0),
            (1, -1, sqrt2),  (1, 0, 1.0),  (1, 1, sqrt2),
        )
        for r, c in ridge_set:
            adj = []
            for dr, dc, edge_w in offsets:
                nbr = (r + dr, c + dc)
                if nbr in ridge_set:
                    adj.append((nbr, edge_w))
            graph[(r, c)] = adj
        return graph

    def _find_nearest_reachable_ridge_cell(
        self, target_row: int, target_col: int, distances: dict
    ) -> Optional[tuple]:
        """Among ridge cells reachable from the robot, find the one nearest (Euclidean) to target."""
        if not distances:
            return None
        keys = list(distances.keys())
        rows = np.array([k[0] for k in keys], dtype=np.int32)
        cols = np.array([k[1] for k in keys], dtype=np.int32)
        dr = rows - target_row
        dc = cols - target_col
        idx = int(np.argmin(dr * dr + dc * dc))
        return keys[idx]

    def _find_nearest_ridge_cell(
        self, target_row: int, target_col: int, ridge_mask: np.ndarray
    ) -> Optional[tuple]:
        """Find the ridge cell closest to (target_row, target_col)."""
        rows, cols = np.where(ridge_mask)
        if len(rows) == 0:
            return None
       
        dr = rows - target_row
        dc = cols - target_col
        dists_sq = dr * dr + dc * dc
        idx = int(np.argmin(dists_sq))
        return (int(rows[idx]), int(cols[idx]))

    def _dijkstra_with_paths(self, start_node: tuple, graph: dict):
        """Shortest path from start_node over the ridge graph, with predecessors.

        Returns (distances, predecessors) — predecessors[node] is the prior
        node along the shortest path from start_node, or None for the start.
        """
        distances = {start_node: 0.0}
        predecessors: dict[tuple, Optional[tuple]] = {start_node: None}
        pq = [(0.0, start_node)]
        while pq:
            d, node = heappop(pq)
            if d > distances[node]:
                continue
            for nbr, edge_w in graph.get(node, []):
                new_d = d + edge_w
                if new_d < distances.get(nbr, float('inf')):
                    distances[nbr] = new_d
                    predecessors[nbr] = node
                    heappush(pq, (new_d, nbr))
        return distances, predecessors

    def _reconstruct_path(self, end_node: tuple, predecessors: dict) -> List[Tuple[int, int]]:
        """Walk predecessor chain back to the start, then reverse."""
        path: List[Tuple[int, int]] = []
        node = end_node
        while node is not None:
            path.append(node)
            node = predecessors.get(node)
        path.reverse()
        return path

    def get_name(self) -> str:
        return 'Voronoi'
