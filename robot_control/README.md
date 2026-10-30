# Robot control guide

The working route command is:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select robot_control
source install/setup.bash
ros2 launch robot_control tag_route.launch.py
```

This starts autonomous control. Add `distance_reporting:=true` to also start the
optional diagnostic distance reporter. The ROS environment and the camera/AprilTag packages must already be available.

## Where to read or edit

| Area | Files | Purpose |
| --- | --- | --- |
| Route settings | [route_config.py](robot_control/route_config.py) | Read the route top to bottom; change target order, final distance, or default motion settings here. |
| Shared configuration | [control_config.py](robot_control/control_config.py) | Common defaults, camera geometry and control timing. |
| Route sequencing | [tag_route.py](robot_control/tag_route.py) | Start each step, wait for success, and switch targets. |
| Shared ROS control | [approach_tag.py](robot_control/approach_tag.py) | Receive TF, check freshness, run the active policy, and publish wheel commands. Also runs standalone. |
| Motion policies | [approach_policy.py](robot_control/approach_policy.py), [square_approach.py](robot_control/square_approach.py), [tag_route_steps.py](robot_control/tag_route_steps.py) | Basic approach/pulse timing, square-on approach geometry, and turn-only alignment respectively. |
| Hardware | [motor_control.py](robot_control/motor_control.py), [kinematic_control.py](robot_control/kinematic_control.py) | Serial bridge and the calibrated wheel/pivot constants used by these policies. |
| Launch | [launch/](launch/) | Starts the camera, detector, reporting, motor bridge and route controller. |
| Tests | [test/](test/) | Hardware-independent motion and route tests plus package lint checks. |
| Detailed trace | [docs/TAG_ROUTE_TRACE.md](docs/TAG_ROUTE_TRACE.md) | Full launch tree, file dependencies, topics, timing, calibration and diagnostics. |

The Python modules stay directly inside `robot_control/` so existing imports and ROS executable entry points continue to work. Documentation lives in `docs/`; launch files, configuration and tests keep their standard ROS package directories.
