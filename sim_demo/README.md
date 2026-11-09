# Sim fallback demo

**Mid-rewrite:** this was written before `ball_pursuit_node` and
`localization_node` got integrated (ball_pursuit_node no longer detects the
goal tag itself -- it looks the goal up from `localization_node`'s known
field map, and gets its own position from `localization_node`'s `/pose`
instead of odom). The script and the notes below describe the OLD,
standalone setup. Being updated now to launch both nodes together.

A Gazebo stand-in for the real field, for when hardware isn't cooperating
(or just to sanity-check a change before trying it on the robot). Spawns
an empty world with:

- an orange ball the robot can detect via HSV color threshold
- a goal: two black posts + a crossbar, with a **real, decodable AprilTag**
  (ID 4, 36h11 family -- generated with `cv2.aruco`, not a placeholder
  image) mounted flush on top of the crossbar

`ball_pursuit_node` finds the ball, finds the tag through the actual rendered
camera feed (same detector code that runs on hardware), and pushes the
ball toward wherever the tag turned out to be.

## Running it

```
bash sim_demo/run_sim_demo.sh
```

Needs `~/ros2_ws` built with `machine_vision_pkg` (`colcon build
--packages-select machine_vision_pkg`) and this repo on `ROS_DOMAIN_ID=23`.
Watch it in the Gazebo GUI window that opens. Ctrl+C in the terminal to
tear everything down.

## Notes / things to know if you're touching this

- The ball and goal are placed on **opposite sides** of the robot's start
  line on purpose, not straight ahead. A straight-line layout hides bugs
  where left/right gets mixed up -- that's exactly how we caught a real
  sign-flip bug in `ball_pursuit_node.py` that was invisible in every earlier
  dead-ahead test.
- The goal structure is colored **black**, not white, deliberately: when
  it was the same white as the tag's quiet-zone border and close to (or
  touching) the tag, `cv2.aruco` failed to decode it even though it looked
  fine to the eye. Keep the goal a different color/contrast from the tag
  backing if you change this.
- The test ball (`orange_ball`) doesn't have realistic rolling resistance
  in sim, so after a strike it can coast much further than a real ball
  would on a real floor -- that's a quirk of this throwaway test prop, not
  of the robot's aiming logic.
- `goal_tag_size` is passed as `0.2` in the launch command here, matching
  this sim prop's actual printed size -- NOT the `0.075` default baked
  into `ball_pursuit_node.py`, which is Sid's measurement of the real field's
  tag. Don't copy that value over to the real robot launch.
