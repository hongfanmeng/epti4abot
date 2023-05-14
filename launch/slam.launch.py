from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    return LaunchDescription(
        [
            # laser driver
            Node(
                name="rplidar_composition",
                package="rplidar_ros",
                executable="rplidar_composition",
                output="screen",
                parameters=[
                    {
                        "serial_port": "/dev/ttyUSB1",
                        "serial_baudrate": 115200,  # A1 / A2
                        "frame_id": "laser",
                        "inverted": False,
                        "angle_compensate": True,
                        "scan_mode": "Standard",
                    }
                ],
            ),
            # slam
            Node(
                name="slam_toolbox",
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                output="screen",
                parameters=[
                    os.path.join(
                        FindPackageShare("epti4abot"),
                        "config",
                        "mapper_params_online_async.yaml",
                    ),
                ],
            ),
            # TF transform
            Node(
                name="tf_base_laser",
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=["0", "0", "0", "0", "0", "0", "base_link", "laser"],
            ),
        ]
    )
