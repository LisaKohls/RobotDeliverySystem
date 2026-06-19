from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    # -------------------------
    # TurtleBot3 base system
    # -------------------------
    tb3_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_bringup'),
                'launch',
                'robot.launch.py'
            ])
        )
    )

    # -------------------------
    # Camera node (frame = physical mount frame)
    # -------------------------
    camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera_top',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('delrobo_bringup'),
                'config',
                'camera.yaml'
            ])
        ]
    )

    # -------------------------
    # TF: base_link → camera
    # -------------------------
    tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '-0.20', '0.0', '0.35',
            '-1.5708', '0.0', '-1.7',
            'base_link',
            'camera'
        ]
    )

    # -------------------------
    # ArUco detector
    # -------------------------
    aruco = Node(
        package='aruco_detector',
        executable='aruco_node',
        output='screen'
    )

    return LaunchDescription([
        tb3_bringup,
        camera,
        tf_camera,
        aruco
    ])
