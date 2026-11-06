"""Temporary demo: overlay everything we know about AprilTags seen by the Neato camera.

Run:  python3 apriltag_viewer.py [--family 36h11] [--tag-size 0.1] [--calib calib.npz]
Press q in the window to quit.
"""
import argparse
import time

import cv2 as cv
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

FAMILIES = {
    '16h5': cv.aruco.DICT_APRILTAG_16h5,
    '25h9': cv.aruco.DICT_APRILTAG_25h9,
    '36h10': cv.aruco.DICT_APRILTAG_36h10,
    '36h11': cv.aruco.DICT_APRILTAG_36h11,
}


class AprilTagViewer(Node):
    def __init__(self, args):
        super().__init__('apriltag_viewer')
        self.family = args.family
        self.tag_size = args.tag_size
        self.hfov = np.radians(args.hfov)
        self.bridge = CvBridge()
        self.frame = None
        self.last_frame_time = None
        self.K, self.D, self.calib_source = None, np.zeros(5), None

        if args.calib:
            data = np.load(args.calib)
            self.K, self.D = data['mtx'], data['dist'].ravel()
            self.calib_source = 'file'

        params = cv.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv.aruco.CORNER_REFINE_APRILTAG
        self.detector = cv.aruco.ArucoDetector(
            cv.aruco.getPredefinedDictionary(FAMILIES[args.family]), params)

        self.create_subscription(Image, '/camera/image_raw', self.on_image, 10)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, 10)

    def on_info(self, msg):
        # The Neato publishes an all-zero CameraInfo when uncalibrated, so only trust a real K.
        if self.calib_source != 'file' and msg.k[0] > 0:
            self.K = np.array(msg.k).reshape(3, 3)
            self.D = np.array(msg.d) if len(msg.d) else np.zeros(5)
            self.calib_source = 'camera_info'

    def on_image(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.last_frame_time = time.monotonic()

    def intrinsics(self, w, h):
        if self.K is not None:
            return self.K, self.D, self.calib_source
        # No calibration: pinhole guess from the assumed horizontal FOV.
        f = (w / 2) / np.tan(self.hfov / 2)
        return np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1]]), np.zeros(5), 'approx'

    def annotate(self, img):
        h, w = img.shape[:2]
        K, D, src = self.intrinsics(w, h)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        s = self.tag_size / 2
        obj_pts = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float32)
        panel = [f'tag{self.family}  tags: {0 if ids is None else len(ids)}  intrinsics: {src}']

        for c, tag_id in zip(corners if ids is not None else [], ids.ravel() if ids is not None else []):
            pts = c.reshape(4, 2)
            center = pts.mean(axis=0)
            area = cv.contourArea(pts)
            sides = [np.linalg.norm(pts[i] - pts[(i + 1) % 4]) for i in range(4)]
            top = pts[1] - pts[0]
            spin = np.degrees(np.arctan2(top[1], top[0]))

            # Outline, corner order, center, id label
            cv.polylines(img, [pts.astype(int)], True, (0, 255, 0), 2)
            for i, (p, col) in enumerate(zip(pts, [(0, 0, 255), (0, 255, 255), (255, 0, 0), (255, 0, 255)])):
                cv.circle(img, tuple(p.astype(int)), 4, col, -1)
                cv.putText(img, str(i), tuple((p + 5).astype(int)), cv.FONT_HERSHEY_SIMPLEX, 0.4, col, 1)
            cv.circle(img, tuple(center.astype(int)), 3, (255, 255, 255), -1)
            cv.putText(img, f'ID {tag_id}', tuple((pts[0] + [0, -10]).astype(int)),
                       cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            panel.append(f'ID {tag_id}: center=({center[0]:.0f},{center[1]:.0f})px  '
                         f'area={area:.0f}px^2  side={np.mean(sides):.1f}px  spin={spin:.0f}deg')

            ok, rvec, tvec = cv.solvePnP(obj_pts, pts, K, D, flags=cv.SOLVEPNP_IPPE_SQUARE)
            if ok:
                cv.drawFrameAxes(img, K, D, rvec, tvec, self.tag_size / 2)
                R, _ = cv.Rodrigues(rvec)
                # ZYX euler angles of the tag in the camera frame
                yaw = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
                pitch = np.degrees(np.arcsin(-np.clip(R[2, 0], -1, 1)))
                roll = np.degrees(np.arctan2(R[2, 1], R[2, 2]))
                t = tvec.ravel()
                bearing = np.degrees(np.arctan2(t[0], t[2]))
                panel.append(f'    xyz=({t[0]:+.2f},{t[1]:+.2f},{t[2]:+.2f})m  dist={np.linalg.norm(t):.2f}m  '
                             f'bearing={bearing:+.0f}deg  rpy=({roll:+.0f},{pitch:+.0f},{yaw:+.0f})deg')

        # Semi-transparent info panel in the top-left
        line_h = 16
        overlay = img.copy()
        width = max(cv.getTextSize(l, cv.FONT_HERSHEY_SIMPLEX, 0.42, 1)[0][0] for l in panel) + 12
        cv.rectangle(overlay, (0, 0), (min(width, w), line_h * len(panel) + 8), (0, 0, 0), -1)
        cv.addWeighted(overlay, 0.6, img, 0.4, 0, img)
        for i, line in enumerate(panel):
            cv.putText(img, line, (6, line_h * (i + 1)), cv.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
        return img


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--family', default='36h11', choices=FAMILIES)
    parser.add_argument('--tag-size', type=float, default=0.1, help='black square edge length in meters')
    parser.add_argument('--hfov', type=float, default=62.2, help='assumed horizontal FOV (deg) if uncalibrated')
    parser.add_argument('--calib', help='.npz with mtx and dist (e.g. np.savez from calibration_demo.py)')
    args, _ = parser.parse_known_args(rclpy.utilities.remove_ros_args()[1:])

    rclpy.init()
    node = AprilTagViewer(args)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            if node.frame is None:
                img = np.zeros((240, 480, 3), np.uint8)
                cv.putText(img, 'waiting for /camera/image_raw ...', (10, 120),
                           cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            else:
                img = node.annotate(node.frame.copy())
                age = time.monotonic() - node.last_frame_time
                if age > 2:
                    cv.putText(img, f'STALE: no new frames for {age:.0f}s', (6, img.shape[0] - 10),
                               cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv.imshow('apriltags', img)
            if cv.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
