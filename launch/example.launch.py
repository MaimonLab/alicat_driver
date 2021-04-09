from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory("alicat_airflow_driver"),
        "config",
        "example.yaml",
    )

    alicat_flow_controller = Node(
        package="alicat_airflow_driver",
        executable="alicat_airflow_server",
        name="airflow_server",
        parameters=[config],
    )
    ld.add_action(alicat_flow_controller)

    test_client = Node(
        package="alicat_airflow_driver",
        executable="example_airflow_client",
        name="airflow_client",
        parameters=[config],
    )
    ld.add_action(test_client)

    return ld