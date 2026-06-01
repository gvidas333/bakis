"""
Exploration orchestrator node.

State machine:
  WAITING_FOR_DATA  -  map or TF not yet available
  COMPUTING_GOAL    -  strategy is calculating the next target
  NAVIGATING        -  goal sent to Nav2, waiting for result
  GOAL_REACHED      -  Nav2 succeeded, ready for next goal
  GOAL_FAILED       -  Nav2 failed/aborted, compute a new goal
  COMPLETE          -  coverage threshold reached or no goals left
"""

import importlib
import math
import time
from enum import Enum, auto

import psutil
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid, Odometry, Path
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import Marker

import tf2_ros

from room_exploration.map_utils import compute_coverage


class State(Enum):
    WAITING_FOR_DATA = auto()
    COMPUTING_GOAL = auto()
    NAVIGATING = auto()
    GOAL_REACHED = auto()
    GOAL_FAILED = auto()
    COMPLETE = auto()


class ExplorationNode(Node):
    def __init__(self):
        super().__init__('exploration_node')

        self.declare_parameter('active_strategy', 'frontier')
        self.declare_parameter('exploration_loop_rate', 1.0)
        self.declare_parameter('goal_timeout', 60.0)
        self.declare_parameter('failure_cooldown', 3.0)
        self.declare_parameter('coverage_threshold', 0.95)
        self.declare_parameter('no_goal_strike_limit', 8)

        self.strategy_name = self.get_parameter('active_strategy').get_parameter_value().string_value
        loop_rate = self.get_parameter('exploration_loop_rate').get_parameter_value().double_value
        self.goal_timeout = self.get_parameter('goal_timeout').get_parameter_value().double_value
        self.failure_cooldown = self.get_parameter('failure_cooldown').get_parameter_value().double_value
        self.coverage_threshold = self.get_parameter('coverage_threshold').get_parameter_value().double_value
        self.no_goal_strike_limit = self.get_parameter('no_goal_strike_limit').get_parameter_value().integer_value

        strategy_params = self._load_strategy_params()

        self.strategy = self._load_strategy(self.strategy_name, strategy_params)
        if not self.strategy:
            self.get_logger().fatal(f'Failed to load strategy "{self.strategy_name}". Node will not run.')
            return

        self.get_logger().info(f'Loaded exploration strategy: {self.strategy.get_name()}')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cb_group = ReentrantCallbackGroup()

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=self.cb_group)
        self.nav_through_poses_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses', callback_group=self.cb_group)

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self._map_callback, map_qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        self.goal_marker_pub = self.create_publisher(Marker, '/exploration/goal_marker', 10)
        self.traveled_path_pub = self.create_publisher(Path, '/exploration/traveled_path', 10)

        self.state = State.WAITING_FOR_DATA
        self.current_map: OccupancyGrid | None = None
        self.traveled_path = Path()
        self.traveled_path.header.frame_id = 'odom'
        self.current_goal_handle = None
        self.goal_send_time: float | None = None
        self._last_goal_pose: PoseStamped | None = None
        self._next_goal_not_before = 0.0

        self.goals_sent = 0
        self.goals_reached = 0
        self.goals_failed = 0
        self.path_length = 0.0
        self.last_odom_pose: Pose | None = None
        self._last_path_pose: Pose | None = None
        self.start_time = time.time()
        self._no_goal_strikes = 0

        self._proc = psutil.Process()
        self._proc.cpu_percent(None)
        self._cpu_count = psutil.cpu_count() or 1
        self._cpu_samples: list[float] = []
        self._mem_samples: list[float] = []

        self.timer = self.create_timer(1.0 / loop_rate, self._exploration_loop)
        self.get_logger().info('Exploration node initialized. Waiting for map and Nav2...')

    def _load_strategy_params(self) -> dict:
        """Read strategy-specific params from YAML into a flat dict.

        ROS 2 requires every parameter to be declared before it can be read,
        and doesn't let us introspect the YAML for keys, so all possible
        strategy+parameter combos are listed here with their defaults.
        """
        param_keys = {
            'strategy_params.frontier.min_frontier_size': 3,
            'strategy_params.frontier.min_goal_distance': 0.3,
            'strategy_params.frontier.clearance_cells': 1,
            'strategy_params.bfs.min_goal_distance': 0.3,
            'strategy_params.bfs.min_frontier_size': 3,
            'strategy_params.bfs.clearance_cells': 1,
            'strategy_params.bfs.goal_standoff_cells': 0,
            'strategy_params.information_gain.sensor_range': 3.5,
            'strategy_params.information_gain.min_frontier_size': 2,
            'strategy_params.information_gain.clearance_cells': 1,
            'strategy_params.information_gain.alpha': 0.5,
            'strategy_params.rrt.step_size': 0.5,
            'strategy_params.rrt.max_iterations': 1000,
            'strategy_params.rrt.goal_bias': 0.1,
            'strategy_params.rrt.sensor_range': 3.5,
            'strategy_params.rrt.cost_weight': 1.0,
            'strategy_params.rrt.clearance_cells': 1,
            'strategy_params.voronoi.sensor_range': 3.5,
            'strategy_params.voronoi.cost_weight': 1.0,
            'strategy_params.voronoi.min_ridge_dist': 2,
            'strategy_params.voronoi.min_frontier_size': 2,
            'strategy_params.voronoi.clearance_cells': 1,
            'strategy_params.voronoi.min_goal_distance': 0.2,
        }

        for key, default in param_keys.items():
            self.declare_parameter(key, default)

        prefix = f'strategy_params.{self.strategy_name}.'
        result = {'coverage_threshold': self.coverage_threshold}
        for key in param_keys:
            if key.startswith(prefix):
                param_name = key[len(prefix):]
                result[param_name] = self.get_parameter(key).value

        return result

    def _load_strategy(self, strategy_name: str, params: dict):
        from room_exploration.exploration.base import BaseExplorationStrategy

        module_path = f'room_exploration.exploration.{strategy_name}'
        try:
            module = importlib.import_module(module_path)
        except ModuleNotFoundError:
            self.get_logger().error(f'Strategy module not found: {module_path}')
            return None

        for attr_name in dir(module):
            attr = getattr(module, attr_name)
            if (
                isinstance(attr, type)
                and issubclass(attr, BaseExplorationStrategy)
                and attr is not BaseExplorationStrategy
            ):
                return attr(self, params)

        self.get_logger().error(f'No BaseExplorationStrategy subclass found in {module_path}')
        return None

    def _map_callback(self, msg: OccupancyGrid):
        self.current_map = msg

    def _odom_callback(self, msg: Odometry):
        pose = msg.pose.pose
        if self.last_odom_pose is not None:
            dx = pose.position.x - self.last_odom_pose.position.x
            dy = pose.position.y - self.last_odom_pose.position.y
            self.path_length += math.hypot(dx, dy)
        self.last_odom_pose = pose

        if self._should_append_path_pose(pose):
            stamped_pose = PoseStamped()
            stamped_pose.header = msg.header
            stamped_pose.pose = pose
            self.traveled_path.header.stamp = msg.header.stamp
            self.traveled_path.poses.append(stamped_pose)
            self._last_path_pose = pose
            self.traveled_path_pub.publish(self.traveled_path)

    def _should_append_path_pose(self, pose: Pose) -> bool:
        if self._last_path_pose is None:
            return True

        dx = pose.position.x - self._last_path_pose.position.x
        dy = pose.position.y - self._last_path_pose.position.y
        return math.hypot(dx, dy) >= 0.03

    def _get_robot_pose(self) -> Pose | None:
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation
            return pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None

    def _exploration_loop(self):
        try:
            self._cpu_samples.append(self._proc.cpu_percent(None))
            self._mem_samples.append(self._proc.memory_info().rss / (1024 * 1024))
        except psutil.Error:
            pass

        coverage = compute_coverage(self.current_map) if self.current_map else 0.0

        if (self.state not in (State.WAITING_FOR_DATA, State.COMPLETE)
                and coverage >= self.coverage_threshold):
            self._finish_exploration(f'Coverage threshold reached ({coverage:.1%})')
            return

        if self.state == State.WAITING_FOR_DATA:
            if self.current_map is None:
                self.get_logger().info('Waiting for /map...', throttle_duration_sec=5.0)
                return
            if self._get_robot_pose() is None:
                self.get_logger().info('Waiting for map-base_footprint TF...', throttle_duration_sec=5.0)
                return
            if not self.nav_to_pose_client.wait_for_server(timeout_sec=0.1):
                self.get_logger().info('Waiting for Nav2 action server...', throttle_duration_sec=5.0)
                return
            if not self.nav_through_poses_client.wait_for_server(timeout_sec=0.1):
                self.get_logger().info('Waiting for Nav2 navigate_through_poses server...', throttle_duration_sec=5.0)
                return
            self.get_logger().info('Map, TF, and Nav2 ready. Starting exploration.')
            self.state = State.COMPUTING_GOAL

        elif self.state in (State.COMPUTING_GOAL, State.GOAL_REACHED, State.GOAL_FAILED):
            self._compute_and_send_goal()

        elif self.state == State.NAVIGATING:
            self._check_navigation_timeout()

        elif self.state == State.COMPLETE:
            pass

    def _compute_and_send_goal(self):
        assert self.strategy is not None
        now = time.time()
        if now < self._next_goal_not_before:
            return

        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            self.get_logger().warn('Lost TF — cannot compute goal.')
            self.state = State.WAITING_FOR_DATA
            return

        self.state = State.COMPUTING_GOAL
        t0 = time.perf_counter()
        path = self.strategy.get_next_path(self.current_map, robot_pose)
        computation_ms = (time.perf_counter() - t0) * 1000

        if not path:
            self._no_goal_strikes += 1
            if self._no_goal_strikes < self.no_goal_strike_limit:
                self.get_logger().info(
                    f'Strategy returned no goal '
                    f'(strike {self._no_goal_strikes}/{self.no_goal_strike_limit}, retrying next tick).'
                )
                self.state = State.COMPUTING_GOAL
            else:
                self._finish_exploration(
                    f'Strategy returned no goal ({self.no_goal_strike_limit} consecutive misses)'
                )
            return

        self._no_goal_strikes = 0

        final_pose = path[-1]

        if len(path) == 1:
            self.get_logger().info(
                f'Sending goal ({final_pose.pose.position.x:.2f}, {final_pose.pose.position.y:.2f}) '
                f'[computed in {computation_ms:.1f}ms]'
            )
            nav_goal = NavigateToPose.Goal()
            nav_goal.pose = final_pose
            send_future = self.nav_to_pose_client.send_goal_async(
                nav_goal, feedback_callback=self._nav_feedback_callback
            )
        else:
            self.get_logger().info(
                f'Sending path of {len(path)} waypoints, final '
                f'({final_pose.pose.position.x:.2f}, {final_pose.pose.position.y:.2f}) '
                f'[computed in {computation_ms:.1f}ms]'
            )
            nav_goal = NavigateThroughPoses.Goal()
            nav_goal.poses = path
            send_future = self.nav_through_poses_client.send_goal_async(
                nav_goal, feedback_callback=self._nav_feedback_callback
            )

        send_future.add_done_callback(self._goal_response_callback)

        self.goal_send_time = time.time()
        self.goals_sent += 1
        self.state = State.NAVIGATING
        self._last_goal_pose = final_pose

        self._publish_goal_marker(final_pose)

    def _publish_goal_marker(self, goal_pose: PoseStamped):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'exploration_goal'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = goal_pose.pose
        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.sec = 0
        self.goal_marker_pub.publish(marker)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2.')
            self.goals_failed += 1
            self.state = State.GOAL_FAILED
            self._next_goal_not_before = time.time() + self.failure_cooldown
            return

        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        if self.state == State.COMPLETE:
            return

        result = future.result()
        # status 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED (action_msgs/GoalStatus)
        if result.status == 4:
            self.get_logger().info('Goal reached!')
            self.goals_reached += 1
            self.state = State.GOAL_REACHED
        else:
            self.get_logger().warn(f'Goal failed with status {result.status}.')
            self.goals_failed += 1
            self.state = State.GOAL_FAILED
            self._next_goal_not_before = time.time() + self.failure_cooldown
            if self._last_goal_pose and self.strategy is not None:
                self.strategy.on_goal_failed(self._last_goal_pose)

        self.current_goal_handle = None

    def _nav_feedback_callback(self, feedback_msg):
        pass

    def _check_navigation_timeout(self):
        if self.goal_send_time is None:
            return
        elapsed = time.time() - self.goal_send_time
        if elapsed > self.goal_timeout:
            self.get_logger().warn(f'Goal timed out after {elapsed:.1f}s. Canceling.')
            if self.current_goal_handle is not None:
                self.current_goal_handle.cancel_goal_async()
            self.goals_failed += 1
            self.state = State.GOAL_FAILED
            self._next_goal_not_before = time.time() + self.failure_cooldown
            self.current_goal_handle = None

    def _finish_exploration(self, reason: str):
        self.state = State.COMPLETE
        elapsed = time.time() - self.start_time
        coverage = compute_coverage(self.current_map) if self.current_map else 0.0

        avg_cpu = sum(self._cpu_samples) / len(self._cpu_samples) if self._cpu_samples else 0.0
        peak_cpu = max(self._cpu_samples) if self._cpu_samples else 0.0
        avg_mem = sum(self._mem_samples) / len(self._mem_samples) if self._mem_samples else 0.0
        peak_mem = max(self._mem_samples) if self._mem_samples else 0.0

        self.get_logger().info(
            f'Exploration complete: {reason}\n'
            f'  Time: {elapsed:.1f}s\n'
            f'  Coverage: {coverage:.1%}\n'
            f'  Path length: {self.path_length:.2f}m\n'
            f'  Goals: {self.goals_sent} sent, {self.goals_reached} reached, {self.goals_failed} failed\n'
            f'  CPU: avg {avg_cpu:.1f}%, peak {peak_cpu:.1f}% '
            f'(across {self._cpu_count} cores; 100% = one full core)\n'
            f'  Memory: avg {avg_mem:.1f} MB, peak {peak_mem:.1f} MB'
        )

    def destroy_node(self):
        if self.current_goal_handle is not None:
            self.get_logger().info('Canceling active goal before shutdown...')
            self.current_goal_handle.cancel_goal_async()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
