# Bags

## field_demo

20s capture from the `sim_demo/` Gazebo field (4 corner tags + 2 goal
tags, ball started centered, robot started near its target goal),
with both `localization_node` and `ball_pursuit_node` running live.
Captures `/odom`, `/pose`, `/cmd_vel`, `/tf`, `/tf_static`, `/scan`,
`/clock`, `/boost`, `/joint_states` -- `/camera/image` is excluded
(raw 1024x1024 RGB at this rate would make the bag enormous for not
much replay value; re-run `sim_demo/run_sim_demo.sh` and watch it live
in Gazebo instead if you need the visual).

Known issue visible in this bag: `/pose.theta` swings across nearly
the full +/-pi range at times. That's not bag/playback noise -- it's
the live localization bug documented in the `ball_pursuit_node.py` /
`localization_node.py` commit history (pose smoothing fixed the x/y
jitter, not the orientation one). `/cmd_vel` reacting erratically to
those swings is expected until that's fixed.
