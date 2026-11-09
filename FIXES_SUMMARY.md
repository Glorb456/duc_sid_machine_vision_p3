# Fixes and Changes Summary (Nov 7-8)

A record of the code changes, calibration work, and debugging done on Sid's
machine on Nov 7-8, so the team can see what changed and what was tried.

> **Machine-specific note.** Everything in the *environment*, *build*, and
> *debugging commands* sections was done on **Sid's computer** (Ubuntu,
> ROS 2 Jazzy, Python 3.12.3, OpenCV 4.10.0, `ROS_DOMAIN_ID=13`, workspace at
> `~/Documents/Sem3/CompRobo/ros2_ws`). Paths, the build location, the domain
> ID, and the installed packages may differ on another computer.
> **The Neato is the same robot regardless of which computer connects to it**,
> so the Neato-side findings (camera resolution, calibration values, topic
> names, driver behavior, bumper/test-mode issues) carry over unchanged.

---

## 1. Code changes

### 1.1 `vision_node.py`: left/right mirror fix (made in this session)
**Problem:** something on the right side of the image was being placed on the
robot's *left*, for both the ball and the goal tag.

- The ball bearing is `atan2(center_x - width/2, f)`, which is **positive when the ball is right of center**.
- The robot frame (ROS convention) is x forward, **y left**.
- The old code used `ball_y_robot = distance * sin(bearing)` and `goal_y_robot = t[0]` (camera x points right), so both were mirrored.
- Because both were mirrored the same way, the ball and goal stayed on the correct sides *relative to each other*. That's why the earlier synthetic check (in commit `8c3eac7`) passed. But the robot then drives using real (unmirrored) odometry, so it would head for the **mirror image** of the real layout. If the robot rotated between seeing the ball and seeing the goal, the two estimates wouldn't even be consistent with each other.

**Change:**
```python
# _find_ball
ball_y_robot = -distance * math.sin(bearing)        # was: distance * math.sin(bearing)

# _find_goal_tag
goal_x_robot, goal_y_robot = float(t[2]), -float(t[0])   # was: float(t[0])
```
The comment above the goal line, which argued for leaving x uninverted, was
replaced with the OpenCV vs robot axis explanation (camera: x right, y down,
z forward; robot: x forward, y left).

**Verified (offline, synthetic):** an orange circle drawn at pixel (650, 300) in
a 768x432 frame (right of center), robot at the origin facing +x:
- locked ball position = `(1.86, -0.81)`, i.e. on the robot's right ✅
- `control_loop` commanded `angular.z = -0.8`, a right turn ✅

Still to do: confirm on the real Neato.

### 1.2 `vision_node.py`: goal tag size
- `GOAL_TAG_SIZE_M` changed from `0.1` (a placeholder) to **`0.075`** (the measured printed tag; matches `localization_node`'s 0.246 ft).
- With the placeholder, every goal distance was 1.33x too far.

### 1.3 `calibration_demo.py` (Sid)
- Removed the cwd-dependent `glob('images/*.jpg')`. Paths are now relative to the file itself:
  - `HERE = os.path.dirname(os.path.abspath(__file__))`
  - reads `HERE/images/*.jpg`, writes `HERE/calibration.npz`
- `ROWS, COLS = 7, 10` constants, used everywhere the pattern size appears (objp, mgrid, find/draw corners).
- Saves `np.savez(OUT_PATH, mtx=mtx, dist=dist, width=w, height=h)` (was `np.savez('images', ...)` with undefined `w`/`h`).
- Guard: exits with a message if fewer than 10 boards are detected (avoids a confusing crash).
- Removed debug prints; the undistort/crop demo is commented out.
- Bugs hit along the way and fixed: `from glob import glob` then calling `glob.glob(...)` (AttributeError); `w, h` undefined after commenting out the line that set them.

### 1.4 `capture_calibration_images.py` (new)
- Live viewer for `/camera/image_raw` that saves frames for calibration.
- SPACE saves the frame only if the chessboard is detected; `f` force-saves; `q`/ESC quits.
- Saves the **raw** frame (not the annotated preview) as `images/calib_NN.jpg`; continues numbering from existing files.
- Run directly with `python3`; no build needed.

### 1.5 `setup.py`
- Fixed the build-breaking `NameError: package_name` (a `data_files` block had been pasted above the `package_name = ...` line). Also fixed by Duc in commit `8c3eac7`.
- Changed approach: dropped `share/<pkg>/config/*.npz` in favor of
  `package_data={package_name: ['calibration.npz']}`, so `calibration.npz` is installed **next to the Python modules** and each node loads it from its own folder. Confirmed it shows up in `install/.../site-packages/machine_vision_pkg/`.
- Status: **modified but not staged.**

### 1.6 `localization_node.py`
- Calibration path: `CALIB_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'calibration.npz')`, loaded with `np.load(CALIB_PATH)` (was a relative `np.load('calibration.npz')`).
- Removed the `get_package_share_directory` approach to match the new `setup.py`.
- `cmd_vel` publisher commented out, so `vision_node` is the only node driving.
- **Still won't start:** `import apriltag` (not installed on this computer), `Vector3` not imported, `self.timer_callback` undefined, publishes on `self.pose_pub`, which is never created. See section 6 of `TODO_machine_vision.md`.

### 1.7 `TODO_machine_vision.md` (new)
Checklist of the remaining work: Done / Demo MVP (Nov 9) / Final MVP (Nov 10) / Stretch.

### Related commits from Duc (made outside this session, listed for context)
- `8c3eac7`: goal tag 4 detection in `vision_node` with `cv2.aruco` + `solvePnP`; `setup.py` NameError fix.
- `b1e26ee`: goal treated as an area (`goal_area_radius` param, default 0.5 m).

---

## 2. Camera calibration (Neato camera, carries over to any computer)

| Item | Value |
|---|---|
| Images | 12 (`images/calib_00..11.jpg`), 12/12 boards detected |
| Pattern | 7x10 inner corners |
| Resolution | **768 x 432** |
| fx, fy | 515.10, 514.40 px |
| cx, cy | 369.25, 217.15 px |
| Distortion | `[-0.0447, 0.1498, 0.0024, -0.0071, -0.0760]` |
| RMS reprojection error | **0.75 px** (recomputed from the images; usable, < 0.5 would be better) |

**About the two "error" numbers:** `calibration_demo.py` printed
`total error: 0.0646`. That isn't the same metric. The script divides the L2
norm over *all* corners by the number of corners, which gives a much smaller
number than the RMS (0.75 px) that `cv.calibrateCamera` returns. Use the RMS
when judging calibration quality.

**What this means for `vision_node` (not changed yet):**
- `CAMERA_FOCAL_LENGTH_PX = 610.18` is from the **simulator**. The real fx is 515, so goal distances come out ~18% too far.
- `image_width / 2` = 384, but the real cx is 369, a ~1.7 deg bearing bias.
- `solvePnP` gets `np.zeros(5)` instead of the measured distortion.
- `BALL_SIZE_CALIBRATION = 61` is also from the simulator.

---

## 3. Environment / build notes (specific to Sid's computer)

- **Workspace was built from `ros2_ws/src/`, not `ros2_ws/`**, so `build/`, `install/`, and `log/` live inside `src/`. Source `ros2_ws/src/install/setup.bash`. Build from the same place every time; don't mix the two.
- Built with `--symlink-install`: `src/build/machine_vision_pkg/machine_vision_pkg` is a **symlink to the source folder**, so editing a file opened from `build/` edits the real source (nothing was lost). Python edits take effect without rebuilding; adding new data files or entry points requires `colcon build`.
- `ros2 run machine_vision_pkg vision_node.py` -> `No executable found`. Use the entry point name **without `.py`**: `ros2 run machine_vision_pkg vision_node`.
- The Python `apriltag` package is **not installed**. `cv2.aruco` (OpenCV 4.10) is used instead in `apriltag_viewer` and `vision_node`.
- `Warning: Ignoring XDG_SESSION_TYPE=wayland on Gnome...` is a harmless Qt message from the OpenCV windows.
- `ROS_DOMAIN_ID=13` on this machine. Any computer talking to this Neato needs the same domain ID.
- **Repo cleanup (Nov 8):** added `.gitignore` (`__pycache__/`, `*.pyc`, `*.egg-info/`, `build/`, `install/`, `log/`, `calibresult.png`); deleted the local `__pycache__` folders; removed the two `.pyc` files that had been committed earlier. A `git rm --cached -r` had accidentally been run on the whole `machine_vision_pkg/machine_vision_pkg/` folder (staging every source file for deletion); the files were never lost on disk and were re-staged.

---

## 4. Neato debugging: "robot sees the ball but doesn't move" (Nov 8)

The robot hardware is the same for everyone, so these findings apply no matter
which computer is connected. The commands were run from Sid's computer.

### 4.1 Symptom
`ros2 run machine_vision_pkg vision_node` was running, the ball was against the
front of the robot and clearly visible in `rqt_image_view` (`/camera/image_raw`),
but the robot didn't move at all, not even the SEARCH spin.

### 4.2 Checks, in order

| # | Command | Result | Conclusion |
|---|---|---|---|
| 1 | `ros2 node list` | `ball_pursuit_node`, `neato_driver`, `gscam_publisher`, `robot_state_publisher`, `fix_scan`, `base_to_laser`, rqt | All expected nodes are up |
| 2 | `ros2 topic info /cmd_vel -v` | 1 publisher (`ball_pursuit_node`), 1 subscriber (`neato_driver`), both `geometry_msgs/msg/Twist` | Wiring is correct |
| 3 | `ros2 topic echo /cmd_vel --once` | `linear.x: 0.2`, `angular.z: -0.005` | Node is in STRIKE, commanding forward. **The node is doing its job** |
| 4 | `ros2 topic hz /cmd_vel` | ~10 Hz | Commands sent continuously |
| 5 | `ros2 topic hz /odom` | ~8 Hz | Driver is reading encoders |
| 6 | `ros2 topic echo /odom --once --field pose.pose` | position **exactly (0, 0, 0)**, orientation (0, 0, 0, 1) | **Wheels haven't turned since the driver started** |
| 7 | `ros2 topic echo /bump --once` | `left_front: 0, left_side: 1, right_front: 0, right_side: 0` | **Left side bumper reads as pressed** (it stayed that way in every check) |
| 8 | Read `neato_node2/neato_node.py` (`cmdVelCb`, main loop) | `cmd_vel` -> `[left, right]` mm/s, clamped to `MAX_SPEED = 300`, sent with `setMotors`. **No software bump/safety check** | The driver isn't what's blocking the motors; the robot is refusing the commands |
| 9 | Stopped `vision_node`, ran `teleop_twist_keyboard` | Robot still **doesn't move**; odom still (0, 0); only `teleop_twist_keyboard` publishing `cmd_vel` | **Rules out our code entirely** |
| 10 | `ros2 topic hz /scan` | **Nothing in 6 s** | **Lidar isn't publishing** |
| 11 | `ros2 topic echo /accel --once` | `accel_z = 0.995` (x, y ~0) | Robot is flat on the floor (not lifted/tilted) |

### 4.3 Diagnosis
- **Data coming out of the robot works:** camera, bump, accelerometer, encoder/odom messages.
- **Commands going into the robot don't:** wheels never move, and the lidar isn't spinning.
- On startup, the driver sends `testmode on` and then `setldsrotation on` (`neato_hybrid_driver.py`, lines 184-186). Motor commands only work in test mode.
- Missing lidar + frozen wheels together point to **the Neato never entering test mode**, meaning it didn't accept the startup commands. Typical causes: the Neato was asleep/off or on the charging dock when the bringup started, or it woke up/reset after the connection was made.

### 4.4 Fixes to try (in order)
1. Ctrl+C teleop/`vision_node` **and the bringup**.
2. Wake the Neato (press the button until the display/light comes on); make sure it's **off the charger**.
3. Restart the bringup and watch for `SETTING TEST MODE TO on` in its output.
4. **Confirm the lidar is spinning** (audible/visible). If it spins, test mode is on and the motors should respond.
5. Test with `ros2 run teleop_twist_keyboard teleop_twist_keyboard` **before** running `vision_node`.
6. If the lidar still doesn't spin after a fresh bringup: check the USB cable between the Pi and the Neato, the battery (low battery can disable the motors/lidar while the sensors still report), or swap to another Neato.
7. Separately: **`left_side` bumper stuck at 1.** Check whether the camera riser/mount presses on the bumper shell. Press and release it by hand and check that `ros2 topic echo /bump` returns to all zeros. This might not be what's stopping the motors, but it needs fixing.

Status at the time of writing: **not yet resolved** (robot still not moving
with teleop).

### 4.5 Node behavior observed (for when the robot does move)
- The node went straight to STRIKE: with the ball touching the bumper, the distance estimate is roughly `61 / radius ≈ 0.15 m`, which still uses the simulator's `BALL_SIZE_CALIBRATION`.
- Unless the visible tag is ID 4, no goal was locked, so the target defaults to `target_x=2.0, target_y=0.0` (straight ahead in the start frame). Fine for a first movement test.

---

## 5. Next steps
See `TODO_machine_vision.md`. The most important ones before the demo:
1. Get the Neato accepting motor commands (section 4.4).
2. Confirm on the real robot that it turns toward the ball (ball right -> turns right).
3. Make `vision_node` load K/D from `calibration.npz` instead of the sim constants.
4. Recalibrate `BALL_SIZE_CALIBRATION` (or switch to ground-plane distance).
5. Unstage `__pycache__`, add a `.gitignore`, commit.
