# alicat airflow driver

This package is a ROS2 wrapper around the alicat python library for serial communcation to the alicat airflow controller.

# Example

The example launch file starts the alicat airflow server and example client. You need to find out which port the alicat is connected to first, which you can do by typing :

    ls /dev/tty*

It is usually at /dev/ttyUSB0, /dev/ttyUSB1, or higher if you have other things inserted into your usb.

To find out which of the usb devices is, type:

    udevadm info  /dev/ttyUSB*

Then look for the vendor "Future Technology Devices International, Ltd", which is apparently the name of the Alicat usb cable.

# installation

    pip3 install alicat
