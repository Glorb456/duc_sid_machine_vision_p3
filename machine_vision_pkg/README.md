# ball_pursuit_node

Drives the Neato up to a ball and pushes it toward a target point. Prototype,
not yet wired into the team repo (`duc_sid_machine_vision_p3`).

## Before this does anything useful

- **Ball color** (`BALL_HSV_LOWER`/`BALL_HSV_UPPER` at the top of the file):
  set for the actual ball, not guessed. Point the camera at it and check the
  real HSV values.
- **Distance calibration** (`BALL_SIZE_CALIBRATION`): there's no depth
  sensor here, distance comes from how big the ball looks. Measure the
  ball's apparent pixel radius at a known distance and set the constant
  from that (see the comment above it in the code).
- **Camera field of view** (`CAMERA_HFOV`): check the real value against
  the camera_info topic if the turning behavior looks off.
- **Target point**: currently a fixed `target_x`/`target_y` ROS parameter
  in the odom frame. This needs to become whatever Sid's AprilTag node
  actually publishes once that exists, instead of a hardcoded guess.

## How it works

Three states: SEARCH (spin until the ball shows up), SETUP (drive to a
point behind the ball, on the far side from the target, so the push sends
it the right way), STRIKE (drive straight through the ball toward the
target once lined up). Ball position in the world frame is estimated from
bearing + distance plus the robot's own odom pose, so the setup/strike
geometry is all done in the odom frame.

## Run it

```bash
python3 ball_pursuit_node.py
```

Needs `camera/image` (sensor_msgs/Image) and `odom` (nav_msgs/Odometry)
publishing, and publishes to `cmd_vel`. Hasn't been run against the sim
yet, next step is testing it in Gazebo and tuning the constants above.
