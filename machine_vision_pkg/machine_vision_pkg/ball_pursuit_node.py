#!/usr/bin/env python3
"""Drives the Neato up to a ball and pushes it toward a target point.

Three states:
  SEARCH  - no ball locked in yet, spin slowly until one shows up
  SETUP   - ball is locked in, drive to a point behind the ball (on the
            far side from the target) so the next push actually sends it
            the right way instead of just batting it sideways
  STRIKE  - lined up behind the ball, drive straight through it toward
            the target

Ball detection is plain HSV color thresholding (no depth sensor assumed).
Distance to the ball is estimated from how big it looks in the image,
using a single calibration constant you need to measure with your own
ball and camera (see BALL_SIZE_CALIBRATION below).

The ball's world-frame position is locked in from the FIRST confident
detection and never recomputed after that, on purpose: the ball is
stationary until struck, so there is no reason to keep re-estimating it.
An earlier version kept re-estimating it on every frame using the
robot's live pose, which turned into a real feedback loop: any small
error in the robot's own heading corrupted the next ball estimate,
which shifted the setup point, which commanded more turning, which added
more heading error, and so on until the robot spiraled away from the
ball entirely. Locking the estimate once removes that loop outright.

The target point is NOT detected live by this node -- it's looked up from
field_config.TAG_POSITIONS, the same known tag map localization_node.py
uses to triangulate the robot's position in the first place. Once you trust
a localization system built on fixed, known landmarks, the goal's position
stops being something to estimate: it's just a constant. (An earlier
version of this file detected AprilTag #4 itself with cv2.aruco + solvePnP,
before localization_node.py existed. That approach still works and is
worth knowing about, but it duplicated geometry localization_node.py
already has more precisely, from a map it already trusts.)

This node's own sense of "where am I" also comes from localization_node.py
now, not odom: it subscribes to the Pose2D it publishes on 'pose', which is
only updated when a tag is in view. Odom drifts without bound; this doesn't,
but it also doesn't update at all while no tag is visible, so robot_pose can
go stale for stretches in the middle of the field. There's no fallback to
odom here -- matching how localization_node.py itself has no fallback.
"""

import math

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose2D, Twist
from rclpy.node import Node
from sensor_msgs.msg import Image

from machine_vision_pkg import field_config

# --- tune these against your actual ball and camera before trusting any of this ---

# HSV range for the ball's color. Defaults assume a bright orange ball;
# point the camera at it in good light and check what cv2.cvtColor gives
# you in HSV, then narrow this range.
BALL_HSV_LOWER = (5, 150, 100)
BALL_HSV_UPPER = (20, 255, 255)
MIN_BALL_CONTOUR_AREA = 50  # pixels, filters out color-matching noise
# A ball detection only gets locked in once two consecutive frames agree
# (within this many meters) on where it is -- guards against a single
# spurious HSV-match frame getting locked in permanently.
BALL_CONFIRM_TOLERANCE = 0.3
BALL_CONFIRM_FRAMES = 2

# distance_m = BALL_SIZE_CALIBRATION / apparent_radius_px
# Measured in the sim: ball at a true camera distance of 1.3m (0.1m
# behind the robot's base plus the ball's own 1.2m from spawn) gave an
# apparent radius of 46.96px, so 1.3 * 46.96 = 61.0. Re-measure this if
# the ball size, camera resolution, or camera mount changes.
BALL_SIZE_CALIBRATION = 61.0

# camera focal length in pixels (the fx entry from /camera/camera_info's
# K matrix), used to turn a pixel offset from image-center into a
# bearing angle via atan2(offset, focal_length). Pulled directly from
# this sim's camera_info; re-check it if the camera model changes.
CAMERA_FOCAL_LENGTH_PX = 610.18 # TODO make this pull from calibration or other method

# Which goal (field_config.TAG_POSITIONS id) to push the ball toward.
# Overridable via the target_goal_id param below without a rebuild.
DEFAULT_TARGET_GOAL_ID = 5

STANDOFF_DISTANCE = 0.35  # meters behind the ball to line up before striking
SETUP_POSITION_TOLERANCE = 0.1  # meters, "close enough" to the setup point
# radians, "facing the right way" before striking. Tightened from an
# earlier 0.15 (~8.6 deg): that was loose enough that the ball could
# visibly miss the target by several tens of centimeters over a few
# meters of push, confirmed in sim testing.
SETUP_HEADING_TOLERANCE = 0.05
# a strike's length is based on how far the ball still needs to go (see
# where STRIKE starts, below), clamped to this range. A fixed-length
# shove massively overshot a light ball in testing: it needs to taper
# off instead of always driving full speed for the same stretch.
MIN_STRIKE_DURATION_SEC = 0.3
MAX_STRIKE_DURATION_SEC = 2.0

MAX_LINEAR_SPEED = 0.2
MAX_ANGULAR_SPEED = 0.8
SEARCH_SPIN_SPEED = 0.4


def angle_diff(a, b):
    """Smallest signed difference a - b, wrapped to [-pi, pi]."""
    d = a - b
    return math.atan2(math.sin(d), math.cos(d))


class BallPursuitNode(Node):

    def __init__(self):
        super().__init__('ball_pursuit_node')

        self.declare_parameter('target_goal_id', DEFAULT_TARGET_GOAL_ID)
        target_goal_id = self.get_parameter('target_goal_id').value
        self.target_xy = field_config.TAG_POSITIONS[target_goal_id]
        # "Scored" means the ball lands anywhere within this radius of the
        # target, not exactly on top of it -- tune-able live without a
        # rebuild since we'll want to adjust this by feel once testing
        # against the real goal.
        self.declare_parameter('goal_area_radius', 0.5)
        self.goal_area_radius = self.get_parameter('goal_area_radius').value

        self.bridge = CvBridge()
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        #camera/image is sim, image_raw is neato
        self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.create_subscription(Pose2D, 'pose', self.pose_callback, 10)

        self.robot_pose = None  # (x, y, yaw) in field frame, from localization_node.py
        self.locked_ball_xy = None  # (x, y) in field frame, set once and never recomputed
        # A single spurious HSV-match frame (sensor noise, a stray
        # reflection) would otherwise get locked in permanently and the
        # robot would chase a phantom for the rest of the run -- confirmed
        # happening in sim testing. Require two consecutive detections
        # that roughly agree before committing to a lock.
        self._ball_candidate_xy = None
        self._ball_candidate_hits = 0

        self.state = 'SEARCH'
        self.strike_start_time = None
        self.strike_duration = None

        self.control_timer = self.create_timer(0.1, self.control_loop)

    def pose_callback(self, msg):
        self.robot_pose = (msg.x, msg.y, msg.theta)

    @staticmethod
    def _project_to_world(x_robot, y_robot, rx, ry, ryaw):
        """Robot-frame (forward, left) offset -> field-frame (x, y)."""
        x = rx + x_robot * math.cos(ryaw) - y_robot * math.sin(ryaw)
        y = ry + x_robot * math.sin(ryaw) + y_robot * math.cos(ryaw)
        return (x, y)

    def _find_ball(self, frame, rx, ry, ryaw):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, BALL_HSV_LOWER, BALL_HSV_UPPER)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < MIN_BALL_CONTOUR_AREA:
            return

        (center_x, _), radius = cv2.minEnclosingCircle(largest)
        if radius <= 0:
            return
        image_width = frame.shape[1]

        pixel_offset = center_x - image_width / 2.0
        bearing = math.atan2(pixel_offset, CAMERA_FOCAL_LENGTH_PX)
        distance = BALL_SIZE_CALIBRATION / radius

        # convert to a world-frame position using the robot's pose right now
        # and lock it in for good. See the module docstring for why this is
        # a one-time, never-recomputed estimate rather than a running one.
        # `bearing` is positive when the ball is RIGHT of image center, but
        # robot-frame +y points LEFT, so the y component is negated.
        ball_x_robot = distance * math.cos(bearing)
        # NEGATED: pixel_offset/bearing positive means the ball is on the
        # RIGHT side of the image, which is the robot's physical right --
        # i.e. NEGATIVE y in the standard x-forward/y-left robot frame that
        # _project_to_world's rotation assumes. Confirmed via an off-axis
        # sim test: an unnegated version locked a ball that was physically
        # placed at y=+0.6 to world y=-0.51 (sign-flipped) -- invisible in
        # all earlier dead-ahead (y=0) testing, where a flipped zero is
        # still zero. (Sid independently hit the same mirrored-driving bug
        # on real hardware and fixed it the same way.)
        ball_y_robot = -distance * math.sin(bearing)
        candidate_xy = self._project_to_world(ball_x_robot, ball_y_robot, rx, ry, ryaw)

        if (self._ball_candidate_xy is not None
                and math.hypot(candidate_xy[0] - self._ball_candidate_xy[0],
                                candidate_xy[1] - self._ball_candidate_xy[1]) < BALL_CONFIRM_TOLERANCE):
            self._ball_candidate_hits += 1
        else:
            self._ball_candidate_xy = candidate_xy
            self._ball_candidate_hits = 1

        if self._ball_candidate_hits >= BALL_CONFIRM_FRAMES:
            self.locked_ball_xy = candidate_xy

    def image_callback(self, msg):
        if self.robot_pose is None or self.locked_ball_xy is not None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        rx, ry, ryaw = self.robot_pose
        self._find_ball(frame, rx, ry, ryaw)

    def control_loop(self):
        cmd = Twist()

        if self.robot_pose is None:
            return

        if self.locked_ball_xy is None:
            self.state = 'SEARCH'
            cmd.angular.z = SEARCH_SPIN_SPEED
            self.cmd_pub.publish(cmd)
            return

        ball_x, ball_y = self.locked_ball_xy
        target_x, target_y = self.target_xy
        rx, ry, ryaw = self.robot_pose

        push_dx = target_x - ball_x
        push_dy = target_y - ball_y
        push_dist = math.hypot(push_dx, push_dy)
        if push_dist < self.goal_area_radius:
            # ball is already within the goal area, no more pushing needed.
            # (Also covers push_dist == 0, where push_dir below would be
            # undefined.)
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return
        push_dir_x, push_dir_y = push_dx / push_dist, push_dy / push_dist

        setup_x = ball_x - STANDOFF_DISTANCE * push_dir_x
        setup_y = ball_y - STANDOFF_DISTANCE * push_dir_y

        to_setup_x, to_setup_y = setup_x - rx, setup_y - ry
        distance_to_setup = math.hypot(to_setup_x, to_setup_y)
        heading_to_setup = math.atan2(to_setup_y, to_setup_x)
        heading_error = angle_diff(heading_to_setup, ryaw)

        push_heading = math.atan2(push_dir_y, push_dir_x)
        strike_heading_error = angle_diff(push_heading, ryaw)

        lined_up = (
            distance_to_setup < SETUP_POSITION_TOLERANCE
            and abs(strike_heading_error) < SETUP_HEADING_TOLERANCE
        )

        if self.state != 'STRIKE' and lined_up:
            self.state = 'STRIKE'
            self.strike_start_time = self.get_clock().now()
            self.strike_duration = max(
                MIN_STRIKE_DURATION_SEC,
                min(MAX_STRIKE_DURATION_SEC, push_dist / MAX_LINEAR_SPEED),
            )

        if self.state == 'STRIKE':
            elapsed = (self.get_clock().now() - self.strike_start_time).nanoseconds / 1e9
            if elapsed < self.strike_duration:
                cmd.linear.x = MAX_LINEAR_SPEED
                # Actively steer toward push_heading during the push itself
                # instead of just coasting on whatever heading SETUP ended
                # on. Gain raised from an earlier 0.3, which let the ball
                # drift noticeably off-line over a multi-meter push.
                cmd.angular.z = 0.8 * strike_heading_error
            else:
                self.state = 'SEARCH'
                self.locked_ball_xy = None  # struck it; look for a new ball next
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
        else:
            self.state = 'SETUP'
            if distance_to_setup < SETUP_POSITION_TOLERANCE:
                # Close enough to the setup point that heading_to_setup (the
                # bearing TO that point) is near-degenerate -- standing
                # right on top of a point gives almost no signal about
                # which way it "is". Pivot to face the strike heading
                # instead, which stays well-defined regardless of position.
                # Without this, a robot that arrives at the setup point
                # facing the wrong way has nothing to correct against and
                # parks there permanently (confirmed in sim testing).
                cmd.angular.z = max(
                    -MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, 1.5 * strike_heading_error))
            else:
                cmd.angular.z = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, 1.5 * heading_error))
                # don't drive forward until mostly facing the setup point,
                # or we'll arc straight through the ball on the way there
                if abs(heading_error) < 0.5:
                    cmd.linear.x = max(0.0, min(MAX_LINEAR_SPEED, 0.5 * distance_to_setup))

        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = BallPursuitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
