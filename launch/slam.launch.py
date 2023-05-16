from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription(
        [
            # laser driver
            Node(
                name="rplidar_node",
                package="rplidar_ros",
                executable="rplidar_node",
                output="screen",
                parameters=[
                    {
                        "serial_port": "/dev/ttyUSB0",
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
                    PathJoinSubstitution(
                        [
                            FindPackageShare("epti4abot"),
                            "config",
                            "mapper_params_online_async.yaml",
                        ]
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
