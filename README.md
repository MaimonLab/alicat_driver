# alicat airflow driver

This package is a ROS2 wrapper around the alicat python library for serial communcation to the alicat massflow controller.

# Installation

Install the python library to interact with the alicat device:

    pip3 install alicat

# Example

The example launch file starts the alicat airflow server and example client.

    ros2 launch alicat_device example.launch.py

This will prompt you to pick one of the config files with parameters. You can open a device by:

    1. A serial number (e.g. FT4TF9NT)
    2. A serial port (e.g. /dev/ttyUSB0)
    3. Neither, the node will try to find a connected device

Using the serial number is most robust as ports are subject to change. You can find out the serial number and port of your device by plugging it in and running:

    ros2 run alicat_device list_ports

Look for a device manufactured by FTDI.

# Release Notes:

**0.1.0** This node is tested with an [2-SLPM Flow Controller](https://store.alicat.com/products/mc-2slpm-d?variant=36749145604249)
