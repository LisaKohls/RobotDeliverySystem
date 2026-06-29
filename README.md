# Robot Delivery System

A prototype robotic delivery system that automates order pickup from a warehouse and delivery to customers using a TurtleBot platform. The system coordinates perception, navigation, item verification and system-level integration to ensure the correct item is picked, secured during transit, and delivered to the correct customer.

> Note: this README documents a ros package for the onboard turtlebot system. Building and running the code requires dependencies (ROS2, OpenCV, ONNX runtime, etc.) and appropriate hardware/simulation.

---

## Overview

Workflow (high level)
1. Customer places an order in the order system.
2. Warehouse places the ordered item onto the TurtleBot at a pickup station marked with ArUco fiducial markers.
3. The TurtleBot uses camera-based ArUco detection and collision-aware navigation to reach pickup and delivery locations.
4. The on-board camera verifies the expected object is loaded and monitors whether the object falls off during transit.
5. The TurtleBot delivers the item to the customer and updates order and database state.

Our group focuses on the TurtleBot itself: camera perception, navigation / drive control, and on-board validation (object verification and fall detection).

---

## Key Capabilities

- Order-driven pickup and delivery workflow.
- ArUco marker detection for precise localization at pickup/drop stations.
- Camera-based object detection and validation.
- Collision detection and obstacle-aware local planning (integrated with TurtleBot navigation stack).
- ROS-native intra-robot communication (ROS topics / nodes).
- System-level coordination via MQTT (order service, robot arm, database, status updates).

---

## Architecture (concise)

- TurtleBot (onboard):
  - ROS nodes: camera / ArUco detector, object detector, navigation controllers, item validator.
  - Topics: image streams, ArUco detections, object verification results, velocity/pose, obstacle alerts.
- Warehouse systems:
  - Robot arm controller to place items on the TurtleBot.
  - Pickup stations identified by ArUco markers.
- Backend & integration:
  - Order service, database, MQTT broker for inter-system messages.

---

## Code reference — perception & bringup examples

The repository contains the ArUco detection node (camera-based marker detection and TF broadcasting). Example excerpt:

```python name=src/aruco_detector/aruco_detector/aruco_node.py url=https://github.com/LisaKohls/RobotDeliverySystem/blob/aedae14411ef7d950506de8ec53995234f0cc62f/src/aruco_detector/aruco_detector/aruco_node.py#L1-L105

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
```
The on-board object detector validates that the expected item is present (publishes detection state and debug images):

```python name=src/cube_detector/cube_detector/detector_node.py url=https://github.com/LisaKohls/RobotDeliverySystem/blob/aedae14411ef7d950506de8ec53995234f0cc62f/src/cube_detector/cube_detector/detector_node.py

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
```

---

## ROS topics (examples)

- Camera / image:
  - /image_raw — raw camera frames (used by ArUco and object detector)
  - /camera_info — camera calibration
- ArUco / localization:
  - /aruco/debug_image — annotated debug frames published by aruco node
  - /aruco_nav/goal_pose — goal pose messages published after marker detection
- Object verification:
  - /cube_detection — detection result (example: "valid_object")
  - /cube_detector/debug_image — debug visualization of detections
- Motion & navigation:
  - /cmd_vel or /turtlebot/cmd_vel — velocity commands (published by controllers)

---

## Getting started (developer notes)

1. Clone the repository and create a ROS2 workspace.
2. Install dependencies: ROS2 (matching repository targets), OpenCV, cv_bridge, ONNX runtime, MQTT broker if integrating inter-system messages.
3. Build the workspace (colcon) and source the install setup.
4. Launch bringup for the TurtleBot + camera + ArUco node, then start the object detector node and navigation stack.
5. Integrate the warehouse systems (robot arm controller) and the order service via an MQTT broker or your preferred bridge.

---

## Responsibilities & Focus

This project’s team focuses on:

- Camera perception (ArUco detection, object recognition)
- Navigation and drive control (local planner, collision avoidance)
- Onboard validation logic (confirm correct load and detect falls during transport)

---

## Next Steps

- Include example MQTT topic schema and sample payloads for system integration.



