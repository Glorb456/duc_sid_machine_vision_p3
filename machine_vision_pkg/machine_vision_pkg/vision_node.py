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

The target point is just a fixed (x, y) parameter in the odom frame for
now. Once Sid's AprilTag node is publishing a real goal position, swap
`self.target_xy` for whatever that node gives you instead of the
hardcoded parameter.
"""

import math

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image

# --- tune these against your actual ball and camera before trusting any of this ---

# HSV range for the ball's color. Defaults assume a bright orange ball;
# point the camera at it in good light and check what cv2.cvtColor gives
# you in HSV, then narrow this range.
BALL_HSV_LOWER = (5, 150, 100)
BALL_HSV_UPPER = (20, 255, 255)
MIN_BALL_CONTOUR_AREA = 50  # pixels, filters out color-matching noise

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
CAMERA_FOCAL_LENGTH_PX = 610.18

STANDOFF_DISTANCE = 0.35  # meters behind the ball to line up before striking
SETUP_POSITION_TOLERANCE = 0.1  # meters, "close enough" to the setup point
SETUP_HEADING_TOLERANCE = 0.15  # radians, "facing the right way" before striking
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


def yaw_from_quaternion(q):
    """Yaw angle from a geometry_msgs/Quaternion, assuming a flat robot."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class BallPursuitNode(Node):

    def __init__(self):
        super().__init__('ball_pursuit_node')

        self.declare_parameter('target_x', 2.0)
        self.declare_parameter('target_y', 0.0)
        self.target_xy = (
            self.get_parameter('target_x').value,
            self.get_parameter('target_y').value,
        )

        self.bridge = CvBridge()
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Image, 'camera/image', self.image_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.robot_pose = None  # (x, y, yaw) in odom frame
        self.locked_ball_xy = None  # (x, y) in odom frame, set once and never recomputed

        self.state = 'SEARCH'
        self.strike_start_time = None
        self.strike_duration = None

        self.control_timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        pose = msg.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)
        self.robot_pose = (pose.position.x, pose.position.y, yaw)

    def image_callback(self, msg):
        if self.robot_pose is None or self.locked_ball_xy is not None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, BALL_HSV_LOWER, BALL_HSV_UPPER)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < MIN_BALL_CONTOUR_AREA:
            return

        (center_x, _), radius = cv2.minEnclosingCircle(largest)
        image_width = frame.shape[1]

        pixel_offset = center_x - image_width / 2.0
        bearing = math.atan2(pixel_offset, CAMERA_FOCAL_LENGTH_PX)
        if radius <= 0:
            return
        distance = BALL_SIZE_CALIBRATION / radius

        # convert to a world-frame position using the robot's pose right now
        # and lock it in for good. See the module docstring for why this is
        # a one-time, never-recomputed estimate rather than a running one.
        rx, ry, ryaw = self.robot_pose
        ball_x_robot = distance * math.cos(bearing)
        ball_y_robot = distance * math.sin(bearing)
        ball_x = rx + ball_x_robot * math.cos(ryaw) - ball_y_robot * math.sin(ryaw)
        ball_y = ry + ball_x_robot * math.sin(ryaw) + ball_y_robot * math.cos(ryaw)
        self.locked_ball_xy = (ball_x, ball_y)

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
        if push_dist < 1e-6:
            # ball is already on the target, nothing useful to push toward
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
                cmd.angular.z = 0.3 * strike_heading_error
            else:
                self.state = 'SEARCH'
                self.locked_ball_xy = None  # struck it; look for a new ball next
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
        else:
            self.state = 'SETUP'
            cmd.angular.z = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, 1.5 * heading_error))
            # don't drive forward until mostly facing the setup point, or
            # we'll arc straight through the ball on the way there
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
