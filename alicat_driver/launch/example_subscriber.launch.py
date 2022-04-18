#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import glob
import ruamel.yaml

ruamel_yaml = ruamel.yaml.YAML(typ="safe")
ruamel_yaml.default_flow_style = False


def generate_launch_description():
    ld = LaunchDescription()

    # locate the config folder path
    workspace = get_package_share_directory("alicat_driver").split("/install")[0]
    config_folder = f"{workspace}/src/alicat_driver/alicat_driver/config"
    config_files = glob.glob(f"{config_folder}/*")

    # print config options
    print(f"\nSelect config file:")
    option_dict = {}
    default_idx = 0
    for idx, selected_config_file in enumerate(config_files):

        config_name = selected_config_file.split("/")[-1]
        if idx == default_idx:
            print(f"   {idx}. {config_name} (default)")
        else:
            print(f"   {idx}. {config_name}")
        option_dict[str(idx)] = selected_config_file

    # prompt user for input
    input_str = input("Select config: ").strip()
    print(f"")

    # parse the input by user
    try:
        if input_str == "":
            input_str = str(default_idx)
        selected_config_file = option_dict[input_str]
        print(f"Launching with config_name: '{selected_config_file}'\n ")
    except:
        print(f"Error parsing your input, exiting")
        exit()

    example_publisher = Node(
        package="alicat_driver",
        executable="example_flowrate_publisher",
        name="flowrate_publisher",
        parameters=[selected_config_file],
    )
    ld.add_action(example_publisher)

    alicat_device = Node(
        package="alicat_driver",
        executable="alicat_device",
        name="alicat_device",
        parameters=[selected_config_file],
    )
    ld.add_action(alicat_device)

    # # if a second_alicat_device is found in the config, add it to the launch
    # with open(selected_config_file, "r") as yaml_file:

    #     example_config = ruamel_yaml.load(yaml_file)
    # if "second_alicat_device" in example_config:
    #     second_mcc_device = Node(
    #         package="alicat_driver",
    #         executable="alicat_device",
    #         name="second_alicat_device",
    #         parameters=[selected_config_file],
    #     )
    #     ld.add_action(second_mcc_device)

    # # if a second_alicat_client is found in the config, add it to the launch
    # if "second_alicat_client" in example_config:
    #     second_mcc_device = Node(
    #         package="alicat_driver",
    #         executable="example_alicat_client",
    #         name="second_alicat_client",
    #         parameters=[selected_config_file],
    #     )
    #     ld.add_action(second_mcc_device)

    return ld
