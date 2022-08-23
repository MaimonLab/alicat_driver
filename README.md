# alicat airflow driver

This package is a ROS2 wrapper around the alicat python library for serial communcation to the alicat massflow controller.

## Installation

Clone the package and run the install script:

    cd ~/maimon_ws/src
    git clone git@github.com:MaimonLab/alicat_driver.git
    cd alicat_driver
    git checkout install-script
    ./install_dependencies.sh

## Use case

The alicat node can receive flowrate either through a service, or through a `goal_flowrate` topic. If you change the flowrate only sparsely, I recommend using the service since is more reliable.
The alicat will publish its measured flowrate once a second regardless, so you can always inspect the current flowrate on the device.

## Example

The example launch file starts the alicat airflow server and example client.

    ros2 launch alicat_device example_server.launch.py

This will prompt you to pick one of the config files with parameters. You can open a device by:

    1. A serial number (e.g. FT4TF9NT)
    2. A serial port (e.g. /dev/ttyUSB0)
    3. Neither, the node will try to find a connected device

Using the serial number is most robust as ports are subject to change. You can find out the serial number and port of your device by plugging it in and running:

    ros2 run alicat_device list_ports

Look for a device manufactured by FTDI.

Similarly, you can launch a publisher/subscriber example:

    ros2 launch alicat_device example_subscriber.launch.py

# Release Notes:

**0.1.0** This node is tested with an [2-SLPM Flow Controller](https://store.alicat.com/products/mc-2slpm-d?variant=36749145604249)
**0.1.1** Publishes state on regular timer.
