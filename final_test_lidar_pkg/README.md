# final_test_lidar_pkg

LiDAR-only escape controller for TurtleBot3 Burger.

The node uses only:

- `/scan` from the LiDAR
- `/odom` from OpenCR wheel odometry
- `/cmd_vel` to drive the base

Run TurtleBot3 bringup first, then:

```bash
ros2 launch final_test_lidar_pkg lidar_escape.launch.py
```

If the robot chooses the rear direction as forward, set `front_angle_offset` in
`config/lidar_escape_params.yaml` to `3.14159` and rebuild or relaunch.
