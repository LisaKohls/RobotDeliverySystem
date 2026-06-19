import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped, PoseStamped

from cv_bridge import CvBridge

import numpy as np
import cv2
from cv2 import aruco

import tf2_ros


class ArucoNavNode(Node):
    def __init__(self):
        super().__init__("aruco_nav_node")

        # ---------------- IO ----------------
        self.bridge = CvBridge()

        self.create_subscription(Image, "/image_raw", self.image_cb, 10)
        self.create_subscription(CameraInfo, "/camera_info", self.info_cb, 10)

        self.img_pub = self.create_publisher(Image, "/aruco/debug_image", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/aruco_nav/goal_pose", 10)

        # ---------------- TF ----------------
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ---------------- Camera state ----------------
        self.K = None
        self.D = None
        self.camera_ready = False

        # ---------------- ArUco ----------------
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters_create()
        self.marker_length = 0.20  # meters

        # ---------------- throttling ----------------
        self._last_warn_time = 0.0

    # ---------------- Camera Info ----------------
    def info_cb(self, msg: CameraInfo):
        if len(msg.k) != 9:
            self.get_logger().warn("Invalid CameraInfo received")
            return

        self.K = np.array(msg.k, dtype=np.float32).reshape(3, 3)
        self.D = np.array(msg.d, dtype=np.float32)
        self.camera_ready = True

    # ---------------- Image ----------------
    def image_cb(self, msg: Image):

        # --- block until calibration exists ---
        if not self.camera_ready:
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self._last_warn_time > 5.0:
                self.get_logger().warn("Waiting for CameraInfo...")
                self._last_warn_time = now
            return

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(
            gray,
            self.dictionary,
            parameters=self.parameters
        )

        if ids is None:
            self._publish_debug(frame, msg)
            return

        aruco.drawDetectedMarkers(frame, corners, ids)

        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners,
            self.marker_length,
            self.K,
            self.D
        )

        for i, marker_id in enumerate(ids.flatten()):
            rvec = rvecs[i]
            tvec = tvecs[i]

            R, _ = cv2.Rodrigues(rvec)
            q = self.rot_to_quat(R)

            # ---------------- TF broadcast ----------------
            tf = TransformStamped()
            tf.header = msg.header
            tf.header.frame_id = "camera_top"
            tf.child_frame_id = f"aruco_{marker_id}"

            tf.transform.translation.x = float(tvec[0][0])
            tf.transform.translation.y = float(tvec[0][1])
            tf.transform.translation.z = float(tvec[0][2])

            tf.transform.rotation.x = q[0]
            tf.transform.rotation.y = q[1]
            tf.transform.rotation.z = q[2]
            tf.transform.rotation.w = q[3]

            self.tf_broadcaster.sendTransform(tf)

            # ---------------- Direct goal publish ----------------
            goal = PoseStamped()
            goal.header.frame_id = "camera_top"
            goal.header.stamp = self.get_clock().now().to_msg()

            goal.pose.position.x = float(tvec[0][0])
            goal.pose.position.y = float(tvec[0][1])
            goal.pose.position.z = 0.0

            goal.pose.orientation.x = q[0]
            goal.pose.orientation.y = q[1]
            goal.pose.orientation.z = q[2]
            goal.pose.orientation.w = q[3]

            self.goal_pub.publish(goal)

            # ---------------- debug axes ----------------
            cv2.drawFrameAxes(
                frame,
                self.K,
                self.D,
                rvec,
                tvec,
                self.marker_length * 0.5
            )

        self._publish_debug(frame, msg)

    # ---------------- debug ----------------
    def _publish_debug(self, frame, msg):
        out = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        out.header = msg.header
        self.img_pub.publish(out)

    # ---------------- quaternion ----------------
    def rot_to_quat(self, R):
        q = np.zeros(4)
        t = np.trace(R)

        if t > 0:
            s = 0.5 / np.sqrt(t + 1.0)
            q[3] = 0.25 / s
            q[0] = (R[2, 1] - R[1, 2]) * s
            q[1] = (R[0, 2] - R[2, 0]) * s
            q[2] = (R[1, 0] - R[0, 1]) * s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
                q[3] = (R[2, 1] - R[1, 2]) / s
                q[0] = 0.25 * s
                q[1] = (R[0, 1] + R[1, 0]) / s
                q[2] = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
                q[3] = (R[0, 2] - R[2, 0]) / s
                q[0] = (R[0, 1] + R[1, 0]) / s
                q[1] = 0.25 * s
                q[2] = (R[1, 2] + R[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
                q[3] = (R[1, 0] - R[0, 1]) / s
                q[0] = (R[0, 2] + R[2, 0]) / s
                q[1] = (R[1, 2] + R[2, 1]) / s
                q[2] = 0.25 * s

        return q


def main():
    rclpy.init()
    node = ArucoNavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
