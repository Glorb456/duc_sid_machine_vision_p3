#!/usr/bin/env python3
""" Drives the Neato to a ball and pushes it toward a target goal.
    States:
        SEARCH: spin in place until a ball is locked in
        SETUP: drive to a point behind the ball, in line with the goal
        STRIKE: drive through the ball toward the goal
    The ball's field position is locked in once and never recomputed, and the
    robot's own pose comes from localization_node. """

import math

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose2D, Twist
from rclpy.node import Node
from sensor_msgs.msg import Image

from machine_vision_pkg import field_config

BALL_HSV_LOWER = (5, 150, 100)
BALL_HSV_UPPER = (20, 255, 255)
MIN_BALL_CONTOUR_AREA = 50  # pixels
BALL_CONFIRM_TOLERANCE = 0.3  # meters between consecutive detections to count as the same ball
BALL_CONFIRM_FRAMES = 2

BALL_SIZE_CALIBRATION = 61.0  # distance_m = BALL_SIZE_CALIBRATION / apparent_radius_px, measured in sim
CAMERA_FOCAL_LENGTH_PX = 610.18 # fx from the sim's camera_info, TODO make this pull from calibration or other method

DEFAULT_TARGET_GOAL_ID = 5

STANDOFF_DISTANCE = 0.35  # meters behind the ball to line up before striking
SETUP_POSITION_TOLERANCE = 0.1  # meters
SETUP_HEADING_TOLERANCE = 0.05  # radians
MIN_STRIKE_DURATION_SEC = 0.3
MAX_STRIKE_DURATION_SEC = 2.0

MAX_LINEAR_SPEED = 0.2
MAX_ANGULAR_SPEED = 0.8
SEARCH_SPIN_SPEED = 0.4


def angle_diff(a, b):
    """ Calculates the smallest signed difference a - b, wrapped to [-pi, pi] """
    d = a - b
    return math.atan2(math.sin(d), math.cos(d))


class BallPursuitNode(Node):

    def __init__(self):
        super().__init__('ball_pursuit_node')

        self.declare_parameter('target_goal_id', DEFAULT_TARGET_GOAL_ID)
        target_goal_id = self.get_parameter('target_goal_id').value
        self.target_xy = field_config.TAG_POSITIONS[target_goal_id]
        self.declare_parameter('goal_area_radius', 0.5)
        self.goal_area_radius = self.get_parameter('goal_area_radius').value

        self.bridge = CvBridge()
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        #camera/image is sim, image_raw is neato
        self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.create_subscription(Pose2D, 'pose', self.pose_callback, 10)

        self.robot_pose = None  # (x, y, yaw) in field frame
        self.locked_ball_xy = None  # (x, y) in field frame
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
        """ Convert a robot frame (forward, left) offset to a field frame (x, y) tuple """
        x = rx + x_robot * math.cos(ryaw) - y_robot * math.sin(ryaw)
        y = ry + x_robot * math.sin(ryaw) + y_robot * math.cos(ryaw)
        return (x, y)

    def _find_ball(self, frame, rx, ry, ryaw):
        """ Detect the ball by HSV threshold and lock in its field position once
            two consecutive detections agree.
                frame: the camera image as a bgr numpy array
                rx, ry, ryaw: the robot's pose in the field frame
        """
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

        ball_x_robot = distance * math.cos(bearing)
        # negated since positive bearing is right of center, but robot frame +y is left
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
        """ Run one step of the SEARCH / SETUP / STRIKE state machine and publish cmd_vel """
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
            # ball is already in the goal area
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
                cmd.angular.z = 0.8 * strike_heading_error
            else:
                self.state = 'SEARCH'
                self.locked_ball_xy = None  # struck it, look for a new ball next
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
        else:
            self.state = 'SETUP'
            if distance_to_setup < SETUP_POSITION_TOLERANCE:
                # at the setup point, so turn to face the strike heading instead
                cmd.angular.z = max(
                    -MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, 1.5 * strike_heading_error))
            else:
                cmd.angular.z = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, 1.5 * heading_error))
                # only drive forward once mostly facing the setup point
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
