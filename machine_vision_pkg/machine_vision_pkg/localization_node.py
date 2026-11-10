""" Estimates the Neato's pose on the field from AprilTags at known positions
    and publishes it on 'pose'. """

import math
import os
import numpy as np
import cv2
import apriltag
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, Pose2D, Point
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge

from machine_vision_pkg import field_config

CALIB_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'calibration.npz')

POSE_SMOOTHING_ALPHA = 0.15  # weight on each new reading, lower is steadier but slower to track real motion


def angle_diff(a, b):
    """ Calculates the smallest signed difference a - b, wrapped to [-pi, pi] """
    d = a - b
    return math.atan2(math.sin(d), math.cos(d))


class localization_node(Node):


    def __init__(self):
        super().__init__('localization_node')
        self.bridge = CvBridge()
        self.detector = apriltag.Detector(apriltag.DetectorOptions(families="tag36h11"))



        #measured parameters in meters
        self.FIELD_SIZE = field_config.FIELD_SIZE
        self.TAG_SIZE = field_config.TAG_SIZE
        self.TAG_HEIGHT = field_config.TAG_HEIGHT
        self.CAMERA_HEIGHT = field_config.CAMERA_HEIGHT

        #boost: if within BOOST_RADIUS of field center, robot may speed up for BOOST_TIME
        self.CENTER = field_config.CENTER
        self.BOOST_RADIUS = 0.10
        self.BOOST_TIME = Duration(seconds=3.0)

        #tag dict: corners 0-3, goals 4-5
        self.TAG_POSITIONS = field_config.TAG_POSITIONS

        calib = np.load(CALIB_PATH)
        mtx = calib['mtx']
        self.camera_params = (mtx[0,0], mtx[1,1], mtx[0,2], mtx[1,2])

        self.pose = Pose2D()
        self.ball = None
        self.boost_until = None
        self._have_pose = False


        self.create_subscription(Image, '/camera/image_raw', self.process_image, 10)
        self.pose_pub = self.create_publisher(Pose2D, 'pose', 10)
        self.boost_pub = self.create_publisher(Bool, 'boost', 10)
        self.get_logger().info('Localization started')


    def process_image(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        detections = self.detect_apriltag(self.cv_image)
        if detections:
            self.estimate_location(detections)
            self.pose_pub.publish(self.pose)
            self.is_on_boost()
        self.boost_pub.publish(Bool(data=self.boosting()))
        self.ball_out_of_field()

    def detect_apriltag(self, cv_image):
        """ Return all detected apriltags whose ids are in TAG_POSITIONS """
        grayscale = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        result = []
        for d in self.detector.detect(grayscale):
            if d.tag_id in self.TAG_POSITIONS:
                result.append(d)
        return result

    def ball_to_msg(self, msg):
        self.ball = msg


    def ground_dist(self, translation_vector):
        """ Convert a camera to tag translation vector into a distance along the floor """
        dist = np.linalg.norm(translation_vector)
        height_diff = self.TAG_HEIGHT - self.CAMERA_HEIGHT

        return math.sqrt(max(dist**2 - height_diff**2, 0.0))


    def estimate_location(self, detections):
        """ Estimate the robot's pose from the nearest detected tag, assuming every
            tag faces the field center, and blend it into self.pose
                detections: list of apriltag detections
        """
        poses = [(d.tag_id, self.detector.detection_pose(d, self.camera_params, self.TAG_SIZE)[0])
                 for d in detections]
        tag_id, pose = min(poses, key=lambda p: np.linalg.norm(p[1][:3, 3]))

        t = pose[:3, 3]        # tag position in camera frame (x right, y down, z forward)
        normal = pose[:3, 2]   # tag's z axis in camera frame
        if np.dot(normal, t) > 0:
            normal = -normal   # point it out of the tag face, toward the camera

        # robot frame is (forward, left) = (cam z, -cam x)
        dist = self.ground_dist(t)
        bearing = math.atan2(-t[0], t[2])                 # direction to the tag, relative to robot heading
        tag_facing_rel = math.atan2(-normal[0], normal[2])  # direction the tag faces, relative to robot heading

        tag_x, tag_y = self.TAG_POSITIONS[tag_id]
        tag_facing = math.atan2(self.CENTER - tag_y, self.CENTER - tag_x)
        theta = tag_facing - tag_facing_rel

        candidate_x = tag_x - dist * math.cos(theta + bearing)
        candidate_y = tag_y - dist * math.sin(theta + bearing)
        candidate_theta = math.atan2(math.sin(theta), math.cos(theta))
        self._smooth_pose(candidate_x, candidate_y, candidate_theta)

    def _smooth_pose(self, x, y, theta):
        """ Blend (x, y, theta) into self.pose with exponential smoothing, so
            alternating readings from two tags settle near their average instead
            of jumping between them """
        if self._have_pose:
            a = POSE_SMOOTHING_ALPHA
            self.pose.x += a * (x - self.pose.x)
            self.pose.y += a * (y - self.pose.y)
            self.pose.theta += a * angle_diff(theta, self.pose.theta)
            self.pose.theta = math.atan2(math.sin(self.pose.theta), math.cos(self.pose.theta))
        else:
            self.pose.x = x
            self.pose.y = y
            self.pose.theta = theta
            self._have_pose = True
            self.get_logger().info('First pose acquired')

    def boosting(self):
        return self.boost_until is not None and self.get_clock().now() < self.boost_until

    def is_on_boost(self):
        """ Start a boost if the robot is within BOOST_RADIUS of the field center """
        dist_to_center = math.hypot(self.pose.x - self.CENTER, self.pose.y - self.CENTER)
        if dist_to_center < self.BOOST_RADIUS and not self.boosting():
            self.boost_until = self.get_clock().now() + self.BOOST_TIME
            self.get_logger().info('at center, boosting for 3 s')


def main(args=None):
    rclpy.init(args=args)
    node = localization_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
