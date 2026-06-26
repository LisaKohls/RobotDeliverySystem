import rclpy
from rclpy.node import Node

import numpy as np
import cv2
import onnxruntime as ort

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

from ament_index_python.packages import get_package_share_directory
from pathlib import Path

from cube_detector.detection import postprocess, draw_detections


class CubeDetectorNode(Node):
    def __init__(self):
        super().__init__("cube_detector_node")

        # ---------------- ROS ----------------
        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            "/image_raw",
            self.image_cb,
            10
        )

        self.detection_pub = self.create_publisher(
            String,
            "/cube_detection",
            10
        )

        self.debug_pub = self.create_publisher(
            Image,
            "/cube_detector/debug_image",
            10
        )

        # ---------------- Model ----------------
        share_dir = Path(get_package_share_directory("cube_detector"))
        model_path = share_dir / "best.onnx"

        if not model_path.exists():
            self.get_logger().error(f"Model not found: {model_path}")
            raise FileNotFoundError(model_path)

        self.get_logger().info(f"Loading ONNX Runtime model: {model_path}")

        self.session = ort.InferenceSession(
            str(model_path),
            providers=["CPUExecutionProvider"]
        )

        self.input_name = self.session.get_inputs()[0].name

        self.get_logger().info(f"ONNX input: {self.input_name}")

    # ---------------- Preprocess ----------------
    def preprocess(self, frame):
        """
        YOLOv8-style preprocessing for ONNX Runtime
        """
        img = cv2.resize(frame, (320, 320))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32) / 255.0

        # HWC → CHW
        img = np.transpose(img, (2, 0, 1))

        # add batch dim
        img = np.expand_dims(img, axis=0)

        return img

    # ---------------- Callback ----------------
    def image_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        input_tensor = self.preprocess(frame)

        # ---------------- Inference ----------------
        outputs = self.session.run(
            None,
            {self.input_name: input_tensor}
        )

        # ---------------- Postprocess ----------------
        detections = postprocess(outputs, frame.shape)

        # ---------------- Publish detection state ----------------
        if detections:
            msg_out = String()
            msg_out.data = "valid_object"
            self.detection_pub.publish(msg_out)

        # ---------------- Debug visualization ----------------
        debug_frame = draw_detections(frame, detections)

        debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8")
        debug_msg.header = msg.header

        self.debug_pub.publish(debug_msg)


def main():
    rclpy.init()
    node = CubeDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
