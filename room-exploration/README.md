# room_exploration

ROS 2 package for autonomous room exploration on a TurtleBot3 Burger.
Compares five exploration strategies on the same SLAM + Nav2 stack.

## Strategies

- `frontier` — nearest frontier
- `bfs` — wavefront BFS from the robot to the first reachable frontier
- `information_gain` — frontiers scored by expected info gain
- `rrt` — RRT through known free space, best-scoring frontier leaf wins
- `voronoi` — walk the GVD ridge toward the best frontier

## Build

```bash
cd ros2_ws
colcon build --packages-select room_exploration
source install/setup.bash
```

## Run

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch room_exploration bringup.launch.py strategy:=frontier
```

Replace `strategy:=` with `bfs`, `information_gain`, `rrt`, or `voronoi`.
