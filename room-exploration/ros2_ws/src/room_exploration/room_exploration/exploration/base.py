import math
from abc import ABC, abstractmethod
from typing import List, Optional, Tuple
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid


class BaseExplorationStrategy(ABC):
    """
    Abstract base class for all exploration strategies.

    Subclasses must implement get_next_goal(). Other methods have sensible
    defaults but can be overridden per algorithm
    """

    def __init__(self, node, params: dict):
        self.node = node
        self.params = params
        self.min_goal_distance = params.get('min_goal_distance', 0.3)
        self.blacklist: List[Tuple[float, float]] = []
        self.blacklist_radius = 0.2

    @abstractmethod
    def get_next_goal(
        self, grid_map: OccupancyGrid, robot_pose: Pose
    ) -> Optional[PoseStamped]:
        pass

    def get_next_path(
        self, grid_map: OccupancyGrid, robot_pose: Pose
    ) -> Optional[List[PoseStamped]]:
        """Return a path of waypoints. Default wraps the single-goal API"""
        goal = self.get_next_goal(grid_map, robot_pose)
        return [goal] if goal else None

    def is_goal_valid(self, wx: float, wy: float, robot_pose: Pose) -> bool:
        rx = robot_pose.position.x
        ry = robot_pose.position.y

        dist = math.hypot(wx - rx, wy - ry)
        if dist < self.min_goal_distance:
            return False

        if any(math.hypot(wx - bx, wy - by) < self.blacklist_radius
               for bx, by in self.blacklist):
            return False

        return True

    def on_goal_failed(self, goal_pose: PoseStamped):
        """Blacklist a goal that Nav2 failed to reach"""
        wx = goal_pose.pose.position.x
        wy = goal_pose.pose.position.y

        self.blacklist.append((wx, wy))

        self.node.get_logger().info(
            f'Blacklisted goal ({wx:.2f}, {wy:.2f}), '
            f'total blacklisted: {len(self.blacklist)}'
        )

    def reset(self) -> None:
        self.blacklist.clear()

    def get_name(self) -> str:
        return self.__class__.__name__
