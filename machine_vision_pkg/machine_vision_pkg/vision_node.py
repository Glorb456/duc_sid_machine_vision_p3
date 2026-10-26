import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
import math

#need opencv
#need camera
#need cmd_vel

class vision_node(Node):
    def __init__(self):
        pass


def main():
    print('Hi from machine_vision_pkg.')


if __name__ == '__main__':
    main()
