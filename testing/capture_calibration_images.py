""" Live viewer for /camera/image_raw that saves raw frames for camera calibration.
    usage: python3 capture_calibration_images.py [--count 15] [--out /some/dir]
    keys:
        SPACE: save the current frame if the chessboard is detected
        f: save the current frame even if the board isn't detected
        q/ESC: quit """
import argparse
import os

import cv2 as cv
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

HERE = os.path.dirname(os.path.abspath(__file__))


class CalibCapture(Node):
    def __init__(self, args):
        super().__init__('calibration_capture')
        self.args = args
        self.pattern = (args.rows, args.cols)
        self.bridge = CvBridge()
        self.frame = None
        os.makedirs(args.out, exist_ok=True)
        existing = [f for f in os.listdir(args.out) if f.startswith('calib_') and f.endswith('.jpg')]
        self.saved = len(existing)
        self.create_subscription(Image, args.topic, self.on_image, 10)
        self.get_logger().info(
            f'Listening on {args.topic}; saving to {args.out} '
            f'({self.saved} already there). SPACE=save, f=force save, q=quit')

    def on_image(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def save(self, img):
        path = os.path.join(self.args.out, f'calib_{self.saved:02d}.jpg')
        cv.imwrite(path, img)
        self.saved += 1
        self.get_logger().info(f'Saved {path} ({self.saved}/{self.args.count})')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--topic', default='/camera/image_raw')
    p.add_argument('--out', default=os.path.join(HERE, 'images'))
    p.add_argument('--count', type=int, default=10, help='stop after this many saved images')
    p.add_argument('--rows', type=int, default=7, help='inner corners (must match calibration_demo.py)')
    p.add_argument('--cols', type=int, default=10, help='inner corners (must match calibration_demo.py)')
    args = p.parse_args()

    rclpy.init()
    node = CalibCapture(args)
    flags = cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_NORMALIZE_IMAGE + cv.CALIB_CB_FAST_CHECK
    try:
        while rclpy.ok() and node.saved < args.count:
            rclpy.spin_once(node, timeout_sec=0.03)
            if node.frame is None:
                cv.waitKey(1)
                continue
            raw = node.frame
            gray = cv.cvtColor(raw, cv.COLOR_BGR2GRAY)
            found, corners = cv.findChessboardCorners(gray, node.pattern, flags)

            preview = raw.copy()
            if found:
                cv.drawChessboardCorners(preview, node.pattern, corners, found)
            status = 'BOARD FOUND' if found else 'no board'
            color = (0, 200, 0) if found else (0, 0, 255)
            cv.putText(preview, f'{status} | saved {node.saved}/{args.count}', (10, 30),
                       cv.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            cv.imshow('calibration capture', preview)

            key = cv.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            if key == ord(' '):
                if found:
                    node.save(raw)
                else:
                    node.get_logger().warn('Board not detected; move it or press f to force-save')
            elif key == ord('f'):
                node.save(raw)
    finally:
        cv.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
