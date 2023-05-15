from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # imu driver
            Node(
                package="epti4abot",
                executable="wit_imu_sensor",
                name="imu_node",
                output="screen",
                parameters=[{"port": "/dev/ttyUSB0", "baudrate": 9600}],
            ),
            # TF transform
            Node(
                name="tf_base_imu",
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=["0", "0", "0", "0", "0", "0", "base_link", "WitSensor"],
            ),
        ]
    )
