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

# calibration.npz lives in the same folder as this file (written by calibration_demo.py)
CALIB_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'calibration.npz')

# estimate_location() computes a fresh pose from a single tag reading each
# call, with no memory of the last one. Normally harmless, but when the
# robot is positioned where two different tags are both the "nearest" one
# on alternating frames, each tag's independent estimate disagrees with the
# other's by a nontrivial amount -- confirmed in testing: a stationary
# robot's reported y swung over a meter, back and forth, frame to frame.
#
# Fix: exponential smoothing. Every new candidate pose is blended with the
# last published one instead of replacing it outright, so two sources that
# disagree converge toward something near their average with a damped
# oscillation, rather than the output swinging all the way to whichever one
# was read most recently. (A first attempt used "hold back a jump until it's
# confirmed by N consecutive agreeing frames" -- that turned out too weak:
# whichever tag happens to stay nearest for just 2 frames in a row, often
# under 0.2s, got waved through as "confirmed," which barely filtered
# anything. Smoothing has no such gap: it always updates, just by a damped
# amount, so it can't be fooled by a short run of one tag's bias, and it
# can't deadlock the way a reject-and-wait scheme can if nothing ever
# re-confirms.)
POSE_SMOOTHING_ALPHA = 0.15  # weight on each new reading; lower = steadier but slower to track real motion


def angle_diff(a, b):
    """Smallest signed difference a - b, wrapped to [-pi, pi]."""
    d = a - b
    return math.atan2(math.sin(d), math.cos(d))


class localization_node(Node):


    def __init__(self):
        super().__init__('localization_node')
        self.bridge = CvBridge()
        self.detector = apriltag.Detector(apriltag.DetectorOptions(families="tag36h11"))



        #measured parameters, all in meters -- shared with ball_pursuit_node.py via
        #field_config.py so the two nodes can't silently disagree about the field
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

        #loads calibration
        calib = np.load(CALIB_PATH)
        mtx = calib['mtx']
        self.camera_params = (mtx[0,0], mtx[1,1], mtx[0,2], mtx[1,2])

        #where it thinks everything is
        self.pose = Pose2D()
        self.ball = None
        self.boost_until = None

        # pose smoothing -- see POSE_SMOOTHING_ALPHA above
        self._have_pose = False


        self.create_subscription(Image, '/camera/image_raw', self.process_image, 10)
        # self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pose_pub = self.create_publisher(Pose2D, 'pose', 10)
        # True while the robot is allowed to drive faster
        self.boost_pub = self.create_publisher(Bool, 'boost', 10)


    def process_image(self, msg):
        # ros image to bgr numpy array
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        detections = self.detect_apriltag(self.cv_image)
        if detections:
            self.estimate_location(detections)
            self.pose_pub.publish(self.pose)
            self.is_on_boost()
        self.boost_pub.publish(Bool(data=self.boosting()))
        self.ball_out_of_field()

        # cv2.imshow('camera', self.cv_image)
        # cv2.waitKey(1)
    
    def detect_apriltag(self, cv_image):
        #returns all apriltag ids (dictionary keys)
        grayscale = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        result = []
        for d in self.detector.detect(grayscale):
            if d.tag_id in self.TAG_POSITIONS:
                result.append(d)
        return result

    def ball_to_msg(self, msg):
        self.ball = msg


    def ground_dist(self, translation_vector):
        dist = np.linalg.norm(translation_vector)
        height_diff = self.TAG_HEIGHT - self.CAMERA_HEIGHT

        #pythagorean theorem, gets floor distance by triangle of distance vector and height diff
        
        return math.sqrt(max(dist**2 - height_diff**2, 0.0))
                

    def estimate_location(self, detections):
        # 4 april tags are in each corner of field, 1 april tag per goal. field is a square.
        #uses one tag assuming all point to center
        poses = [(d.tag_id, self.detector.detection_pose(d, self.camera_params, self.TAG_SIZE)[0])
                 for d in detections]
        tag_id, pose = min(poses, key=lambda p: np.linalg.norm(p[1][:3, 3]))

        t = pose[:3, 3]        # tag position in camera frame (x right, y down, z forward)
        normal = pose[:3, 2]   # tag's z axis in camera frame
        if np.dot(normal, t) > 0:
            normal = -normal   # make it point out of the tag face, toward the camera

        # robot frame is (forward, left) = (cam z, -cam x)
        dist = self.ground_dist(t)
        bearing = math.atan2(-t[0], t[2])                 # direction to the tag, relative to robot heading
        tag_facing_rel = math.atan2(-normal[0], normal[2])  # direction the tag faces, relative to robot heading

        # we know which way the tag faces on the field, so that gives us the robot's heading
        tag_x, tag_y = self.TAG_POSITIONS[tag_id]
        tag_facing = math.atan2(self.CENTER - tag_y, self.CENTER - tag_x)
        theta = tag_facing - tag_facing_rel

        # step back from the tag along the line of sight
        candidate_x = tag_x - dist * math.cos(theta + bearing)
        candidate_y = tag_y - dist * math.sin(theta + bearing)
        candidate_theta = math.atan2(math.sin(theta), math.cos(theta))
        self._smooth_pose(candidate_x, candidate_y, candidate_theta)

    def _smooth_pose(self, x, y, theta):
        """Blend (x, y, theta) into self.pose instead of replacing it outright
        -- see POSE_SMOOTHING_ALPHA above for why."""
        if self._have_pose:
            a = POSE_SMOOTHING_ALPHA
            self.pose.x += a * (x - self.pose.x)
            self.pose.y += a * (y - self.pose.y)
            self.pose.theta += a * angle_diff(theta, self.pose.theta)
            self.pose.theta = math.atan2(math.sin(self.pose.theta), math.cos(self.pose.theta))
        else:
            # first-ever reading: no prior estimate to blend with, just take it
            self.pose.x = x
            self.pose.y = y
            self.pose.theta = theta
            self._have_pose = True

    def boosting(self):
        return self.boost_until is not None and self.get_clock().now() < self.boost_until

    def is_on_boost(self):
        # if the robot thinks it is within 10cm of the center, allow it to speed up for 3 seconds
        dist_to_center = math.hypot(self.pose.x - self.CENTER, self.pose.y - self.CENTER)
        if dist_to_center < self.BOOST_RADIUS and not self.boosting():
            self.boost_until = self.get_clock().now() + self.BOOST_TIME
            self.get_logger().info('at center, boosting for 3 s')

    def ball_out_of_field(self):
        #TODO if neato thinks ball is outside of field, spin around in a circle and keep verifying surroundings until it is back in
        pass




def main(args=None):
    rclpy.init(args=args)
    node = localization_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
