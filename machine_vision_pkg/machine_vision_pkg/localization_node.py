import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import math
import cv2
import apriltag


class localization_node(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.bridge = CvBridge()
        self.cv_image = None
        self.grayscale = None
        self.detector = apriltag.Detector(apriltag.DetectorOptions(families="tag36h11"))


        self.create_subscription(Image, '/camera/image_raw', self.process_image, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def process_image(self, msg):
        # Convert ROS Image -> OpenCV BGR numpy array
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow('camera', self.cv_image)
        cv2.waitKey(1)
    
    def detect_apriltag(self):
        self.grayscale = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        self.detector.detect(estimate_tag_pose=True)

    def estimate_location(self):
        # 4 april tags are in each corner of field, 1 april tag per goal. field is a square
        pass

    def is_on_boost(self):
        #if the robot is on the center of the map, double the speed for 3 seconds
        pass

    def ball_out_of_field(self):
        #if neato thinks ball is outside of field, spin around in a circle and keep verifying surroundings until it is back in
        pass




def main(args=None):
    rclpy.init(args=args)
    node = vision_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
