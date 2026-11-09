# Machine Vision Project: Remaining Work

Deadlines: **demo Nov 9 (in class)**, **code + README Nov 10, 7 PM**.
Tags: `[no robot]` = can be done without the Neato/camera, `[robot]` = needs the Neato.

_Last updated Nov 8 after reviewing the repo (commits through `b1e26ee` plus uncommitted work)._

---

## Decide first

- [ ] **Which pipeline do we demo on Nov 9?**
  - **A. Odom frame + goal tag lock** (what `vision_node.py` does now): find the ball, find tag 4, lock both in odom coordinates, push. No field localization needed. Closest to working.
  - **B. Full field localization** (`localization_node.py` -> `field_pose` -> `vision_node`): the original plan. Most of it isn't written yet.
  - Suggestion: get **A** working for the Nov 9 demo (sections 1-3 below), then build **B** for the Nov 10 code/README (sections 5-7) and present B as "in progress" if it isn't done.

---

## Done

- [x] `setup.py` build fixed (`package_name` NameError), commit `8c3eac7`.
- [x] `nav_msgs` added to `package.xml`.
- [x] `vision_node` subscribes to `camera/image_raw` (the real Neato topic).
- [x] `capture_calibration_images.py`: live preview, only saves frames where the board is detected.
- [x] `calibration_demo.py` cleaned up: paths relative to the file, saves `calibration.npz` with `mtx`, `dist`, `width`, `height`.
- [x] Real calibration: 12/12 images detected at **768x432**. fx=515.1, fy=514.4, cx=369.3, cy=217.2, **RMS 0.75 px** (usable).
- [x] `calibration.npz` installed through `package_data` (it shows up in `install/`).
- [x] `vision_node` detects goal tag 4 with `cv2.aruco` + `solvePnP` and locks the goal position.
- [x] Goal treated as an area (`goal_area_radius` param).
- [x] `localization_node` no longer publishes `cmd_vel` (commented out), so `vision_node` is the only node driving.

---

## MVP: Demo (Nov 9), pipeline A

### 1. Repo hygiene `[no robot]`
- [ ] Commit the untracked work: `calibration.npz`, `images/`, `capture_calibration_images.py`, and the modified `calibration_demo.py` / `setup.py` / `localization_node.py`.
- [ ] Add a `.gitignore` with `__pycache__/` (pycache folders are showing up as untracked).
- [ ] `colcon build --symlink-install` passes; `ros2 run machine_vision_pkg vision_node` starts.
- [ ] (Optional) Add 5-10 more calibration images (tilted, board in the frame corners) to get RMS below 0.5 px.

### 2. `vision_node.py` fixes: these directly affect whether the demo works
- [x] **Left/right sign bug** (fixed Nov 8; synthetic test: ball right of center -> y < 0, robot turns right). Image-right is mapped to robot **+y (left)** for both the ball (`ball_y_robot = d*sin(bearing)` with `bearing` positive to the right) and the goal (`goal_y_robot = t[0]`). Because both are flipped the same way, they land on the correct sides *relative to each other*, but the robot physically drives to the **mirror image** of where they really are. If the robot rotated between seeing the ball and seeing the goal, they aren't even consistent with each other. Fix: negate both (`-sin(bearing)`, `-t[0]`).
  - [ ] `[robot]` Confirm on the real Neato: ball on the right side of the image -> the robot turns **right** (`angular.z < 0`).
- [ ] **Use `calibration.npz` instead of the sim constants.** `CAMERA_FOCAL_LENGTH_PX = 610` comes from the sim; the real fx is **515**, so goal distances are currently ~18% too far. `image_width/2` = 384 but the real cx is **369**, a ~1.7 deg bearing bias. `solvePnP` gets `np.zeros(5)` instead of the real distortion `D`.
- [ ] **`GOAL_TAG_SIZE_M = 0.1` is a placeholder.** `localization_node` has 0.246 ft = **0.075 m**. Measure the black square and set it. At 0.1 vs 0.075, distances come out 1.33x too far.
- [ ] **Ball distance.** `BALL_SIZE_CALIBRATION = 61` is from the sim. Quick fix `[robot]`: measure the pixel radius at a known distance, then constant = distance x radius.
- [ ] `[robot]` Tune `BALL_HSV_LOWER/UPPER` on the real ball under the room's lighting; add morphological open/close to the mask.
- [ ] Update the module docstring (it says `localization_node` uses the `apriltag` pip package and describes the ball distance method).

### 3. Demo run `[robot]`
- [ ] Ball on the right / left / center -> robot turns the correct way (confirms the sign fix).
- [ ] Goal tag at a measured distance -> locked goal position is roughly right (log it).
- [ ] Full run: ball + goal tag 4 -> robot sets up behind the ball and pushes it toward the goal. Record N tries and how many scored.
- [ ] **Record video** of the best runs, plus a screen capture of `apriltag_viewer` for the slides.
- [ ] Presentation (~10 min): motivation, approach (calibration -> PnP -> frames -> ball detection -> FSM), demo video, limitations, real-world uses, lessons learned. Submit on Canvas.

---

## MVP: Final code + README (Nov 10), pipeline B

### 4. Shared calibration loader `[no robot]`
- [ ] `camera_utils.py` with `load_calibration()` that reads `calibration.npz` next to the module (the convention the team already chose) and returns `K`, `D`, `(width, height)`.
- [ ] Use it in `vision_node`, `localization_node`, and `apriltag_viewer`; log an error if the incoming image isn't 768x432.

### 5. Physical measurements (all in **meters**) `[robot]`
The values in `localization_node.py` are currently **in feet** (`TAG_SIZE = 0.246` ft = 0.075 m, `TAG_HEIGHT = 0.208` ft, `CAMERA_HEIGHT = 0.875` ft).
- [ ] Tag size (outer black square edge): confirm 0.075 m.
- [ ] Tag center height above the floor.
- [ ] Camera height above the floor (with the riser).
- [ ] Camera forward offset from the robot center.
- [ ] Camera pitch (downward tilt).
- [ ] Field size: is `FIELD_SIZE = 4.0` meters or feet?
- [ ] For each tag ID: field (x, y) **and the direction it faces (yaw)**.
- [ ] Ball diameter.

### 6. `localization_node.py`: single-tag pose
Currently **won't start**: `import apriltag` (not installed), `Vector3` not imported, `self.timer_callback` doesn't exist, publishes on `self.pose_pub`, which is never created.
- [ ] Remove `import apriltag`; detect with `cv2.aruco` (36h11), the same as `vision_node` / `apriltag_viewer`.
- [ ] Load `K`, `D` with `load_calibration()` (`camera_params` is the format for the `apriltag` library and isn't needed).
- [ ] Publisher: `Pose2D` on `field_pose`, one name; remove the timer; use the `cv_image` argument in `detect_apriltag` (it reads `self.cv_image`).
- [ ] Convert constants to meters; change `TAG_POSITIONS` to `(x, y, yaw)`.
- [ ] Move `is_on_boost` / `ball_out_of_field` / `ball_to_msg` out (behavior node or stretch).
- [ ] Transform chain: `solvePnP` -> T_cam_tag -> invert -> T_field_tag * T_tag_cam * T_cam_robot -> robot (x, y, theta).
- [ ] Handle OpenCV camera axes (x right, y down, z forward) vs robot axes (x forward, y left, z up).
- [ ] **Synthetic test `[no robot]`:** pick a camera pose, `projectPoints` the tag corners with the real K, run the pipeline, check it recovers the pose.
- [ ] Basic rejection: ignore poses outside the field or large jumps.
- [ ] `[robot]` Accuracy test: 3-5 taped spots, estimated vs true pose (for the README).

### 7. Odometry bridging
- [ ] Subscribe to `odom` in `localization_node`.
- [ ] On a tag sighting: `T_field_odom = T_field_robot * inv(T_odom_robot)`.
- [ ] On each odom message: publish `field_pose = T_field_odom * odom_pose`.
- [ ] Use the odom pose closest to the image's timestamp.
- [ ] `[robot]` Drive around; the pose stays sensible when tags drop out of view.

### 8. Integration
- [ ] `vision_node` uses `field_pose` instead of raw `odom`.
- [ ] Goal position comes from `TAG_POSITIONS[goal_id]` (param: 4 or 5) instead of detecting/locking it in `vision_node`.
- [ ] Ball position computed in the field frame.
- [ ] LOCALIZE state: spin until there's a field pose, then SEARCH.
- [ ] Launch file starting `localization_node` + `vision_node` with params (goal ID, etc.).
- [ ] `[robot]` End-to-end run; record attempts / success rate.

### 9. Writeup + submission
- [ ] Top-level `README.md`: answer all 7 headings (currently empty), with images, a frames diagram, calibration results (RMS 0.75 px, K), and accuracy tests.
- [ ] Replace or delete `machine_vision_pkg/README.md` (out of date: describes `ball_pursuit_node.py`, `CAMERA_HFOV`, and the `camera/image` topic).
- [ ] Embed the demo video / GIFs.
- [ ] Clean up debug prints; document `apriltag_viewer` and the calibration scripts as tools.
- [ ] Push to GitHub, submit the link on Canvas.

---

## Stretch

- [ ] **Boost zone:** circle at the field center gives 2x speed for 3 s, with a cooldown so it doesn't re-trigger every frame (check the Neato's max speed).
- [ ] **Ball out of field:** if the ball's field estimate is outside [0, F]^2, spin and re-verify until it's back in.
- [ ] **Ground-plane ball distance:** pixel ray (via `undistortPoints`) + camera height + pitch, intersected with the plane z = ball radius. Handles a ball cut off at the edge of the frame.
- [ ] **Multi-tag fusion:** weighted average of the poses from all visible tags (circular mean for yaw; weight by distance/size).
- [ ] **PnP ambiguity handling:** use both IPPE solutions and pick the one most consistent with odom.
- [ ] Smooth the `T_field_odom` corrections (blend instead of snapping).
- [ ] Re-observe the ball while approaching, with a simple filter, instead of locking it once.
- [ ] Debug visualization: top-down field plot or RViz markers (robot, ball, goals, setup point).
- [ ] Publish a debug image topic with the tag/ball overlays.
- [ ] Record rosbags and tune offline.
- [ ] Smarter strike: re-aim if the ball is between the robot and the wrong goal.
- [ ] Two-robot / defender mode.
