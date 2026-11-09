import math
import os
import numpy as np
import cv2 
import apriltag
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D, Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# calibration.npz lives in the same folder as this file (written by calibration_demo.py)
CALIB_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'calibration.npz')


class localization_node(Node):


    def __init__(self):
        super().__init__('localization_node')
        self.bridge = CvBridge()
        self.detector = apriltag.Detector(apriltag.DetectorOptions(families="tag36h11"))



        #measured parameters in ft, need to translate to meters
        self.FIELD_SIZE = 4.0
        self.TAG_SIZE = 0.246063
        self.TAG_HEIGHT = 0.2083333
        self.CAMERA_HEIGHT = 0.875

        #tag dict
        self.TAG_POSITIONS = {

            #corners
            0: (0.0, 0.0),
            1: (self.FIELD_SIZE, 0.0),
            2: (self.FIELD_SIZE, self.FIELD_SIZE),
            3: (0.0, self.FIELD_SIZE),

            #goals
            4: (0.0, self.FIELD_SIZE/2), 
            5: (self.FIELD_SIZE, self.FIELD_SIZE/2)
        }

        #loads calibration
        calib = np.load(CALIB_PATH)
        mtx = calib['mtx']
        self.camera_params = (mtx[0,0], mtx[1,1], mtx[0,2], mtx[1,2])

        #where it thinks everything is
        self.pose = Pose2D()
        self.ball = None
        self.boost_until = None


        self.create_subscription(Image, '/camera/image_raw', self.process_image, 10)
        # self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.publisher_ = self.create_publisher(Vector3, 'pose', 10) #need to publish x,y,theta, this is probably wrong
        self.timer = self.create_timer(1.0, self.timer_callback)


    def process_image(self, msg):
        # ros image to bgr numpy array
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        detections = self.detect_apriltag(self.cv_image)
        if detections:
            self.estimate_location(detections)
            self.pose_pub.publish(self.pose)
        self.is_on_boost()
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
        # TODO 4 april tags are in each corner of field, 1 april tag per goal. field is a square
        pass

    def is_on_boost(self):
        # TODO if the robot is on the center of the map, double the speed for 3 seconds
        pass

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
