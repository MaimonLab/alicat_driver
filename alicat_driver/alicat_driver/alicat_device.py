#!/usr/bin/env python3

"""alicat_device
ROS2 node to send flowrate on an alicat flow rate device. 
Parameters: 
- device_port: port, e.g. /dev/ttyUSB0 to open the device. Serial number is prefered since ports change.
- device_serial_number: serial number of the device to open, takes precedence over device_port.
- subscribe_flowrate: option to open a subscription, if false the flowrate can only be set with services. 
- flowrate_topic: topic name that the node will subscribe to. 
- flowrate_service: service name that the node will provide. 
"""

import rclpy
from rclpy.node import Node
from alicat import FlowController
from alicat_driver_interfaces.srv import SetFlowRate
from alicat_driver_interfaces.msg import FlowRate
import time
import serial.tools.list_ports
from typing import Optional
from contextlib import contextmanager
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from event_data_logging import CSVWriter
import signal


def find_port_for_serial(serial_id: str) -> Optional[str]:
    """Returns port string if found, otherwise returns None"""

    ports = list(serial.tools.list_ports.comports())

    for port in ports:
        if port.serial_number == serial_id:
            return port.device

    return None


def get_alicat_port():
    """Loops through ports and sees if any device is manufactured by FTDI,
    which is the chip which the alicat comports devices use"""

    ports = list(serial.tools.list_ports.comports())

    for port in ports:
        if port.manufacturer == "FTDI":
            return port.device
    return None


def find_alicat_serial_from_port(flow_controller):
    """For an opened alicat device, find it's serial by finding what serial number
    and manufacturer is associated with it's port"""

    flow_controller_data = flow_controller.open_ports
    for key, item in flow_controller_data.items():

        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            if (port.device == item[0].port) & (port.manufacturer == "FTDI"):
                found_serial_number = port.serial_number
                return found_serial_number
    return None


class AlicatNode(Node):
    def __init__(self):
        super().__init__("alicat_node")
        default_param = {
            "device_port": None,
            "device_serial_number": None,
            "goal_flowrate_topic": "alicat/goal_flowrate",
            "measured_flowrate_topic": "alicat/measured_flowrate",
            "flowrate_service": "set_flow_rate",
            "output_filename": None,
            "save_data_to_csv": True,
            "csv_saving_rate_hz": 5,
        }

        for key, value in default_param.items():
            if not self.has_parameter(key):
                self.declare_parameter(key, value)

        self.port_from_param = self.get_parameter("device_port").value
        self.device_serial_number = self.get_parameter("device_serial_number").value

        if self.device_serial_number:
            self.device_serial_number = self.device_serial_number.replace("$", "")

        with self.exit_node_if_unable_to_open():
            if self.device_serial_number:
                port_to_try = find_port_for_serial(self.device_serial_number)
                if port_to_try is None:
                    raise FileNotFoundError(b"Cannot find port for serial")
                self.flow_controller = FlowController(port_to_try)
                self.get_logger().info(
                    f"Opened Alicat w s/n: {self.device_serial_number} on {port_to_try}"
                )
            elif self.port_from_param:
                self.flow_controller = FlowController(self.port_from_param)
                found_serial_number = find_alicat_serial_from_port(self.flow_controller)
                if found_serial_number is None:
                    raise FileNotFoundError(b"Cannot find alicat device")
                self.get_logger().info(
                    f"Opened Flow controller in port: {self.port_from_param} with s/n {found_serial_number}"
                )
            else:
                ports = list(serial.tools.list_ports.comports())
                port = get_alicat_port()

                if port is None:
                    raise FileNotFoundError(b"Cannot find alicat device")

                self.flow_controller = FlowController(ports[0].device)

                found_serial_number = find_alicat_serial_from_port(self.flow_controller)

                self.get_logger().warn(
                    f"No port or serial number specified. Opening device with serial {found_serial_number}"
                )

        flowrate_service = self.get_parameter("flowrate_service").value

        self.create_service(
            SetFlowRate, flowrate_service, self.flowrate_service_callback
        )

        qos_subscriber = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        goal_flowrate_topic = self.get_parameter("goal_flowrate_topic").value
        self.create_subscription(
            FlowRate, goal_flowrate_topic, self.goal_flowrate_callback, qos_subscriber
        )

        qos_subscriber_best_effort = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        goal_flowrate_topic = self.get_parameter("goal_flowrate_topic").value
        self.create_subscription(
            FlowRate,
            goal_flowrate_topic,
            self.goal_flowrate_callback,
            qos_subscriber_best_effort,
        )

        qos_publisher_best_effort = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        measured_flowrate_topic = self.get_parameter("measured_flowrate_topic").value
        self.pub_flowrate = self.create_publisher(
            FlowRate, measured_flowrate_topic, qos_publisher_best_effort
        )

        self.flow_controller.set_flow_rate(0.0)

        if (
            self.get_parameter("output_filename").value
            and self.get_parameter("save_data_to_csv").value
        ):
            topic_extension = measured_flowrate_topic.split("/")[-1]
            output_filename = (
                f"{self.get_parameter('output_filename').value}_{topic_extension}.csv"
            )

            self.csv_writer = CSVWriter(
                output_filename,
                header=["timestamp", "massflow", "pressure", "temperature"],
            )
        else:
            self.get_logger().warn(f"No output filename specified, not saving data")
            self.csv_writer = None

        self.flow_controller.set_flow_rate(0.0)

        csv_saving_rate = self.get_parameter("csv_saving_rate_hz").value
        self.create_timer(1 / csv_saving_rate, self.measure_flowrate_callback)

        signal.signal(signal.SIGINT, self.signal_interrupt)

    def signal_interrupt(self, frame, what):
        # catch signal to explicitly call shutdown while node is still alive
        self.on_shutdown()
        self.destroy_node()
        exit()

    def on_shutdown(self):
        self.flow_controller.set_flow_rate(0.0)
        self.get_logger().info(f"Alicat flowrate set to 0")

    def goal_flowrate_callback(self, goal_flowrate_msg):
        """Callback for topic subscription of the flowrate message"""

        raw_flowrate = goal_flowrate_msg.flowrate

        # truncate negative flowrate requests
        if raw_flowrate < 0:
            apply_flowrate = 0
        else:
            apply_flowrate = raw_flowrate

        self.flow_controller.set_flow_rate(apply_flowrate)

    def measure_flowrate_callback(self):
        """Callback for topic subscription of the flowrate message"""

        actual_flowrate_msg = FlowRate()
        # actual_flowrate_msg.header = goal_flowrate_msg.header
        actual_flowrate_msg.header.stamp = self.get_clock().now().to_msg()

        flow_status = self.flow_controller.get()
        actual_flowrate_msg.flowrate = flow_status["mass_flow"]
        actual_flowrate_msg.pressure = flow_status["pressure"]
        actual_flowrate_msg.temperature = flow_status["temperature"]
        self.pub_flowrate.publish(actual_flowrate_msg)

        data_line = [
            rclpy.time.Time.from_msg(actual_flowrate_msg.header.stamp).nanoseconds,
            flow_status["mass_flow"],
            flow_status["pressure"],
            flow_status["temperature"],
        ]
        if self.csv_writer:
            self.csv_writer.save_line(data_line)

    def flowrate_service_callback(self, request, response):
        """Callback for the service call set_flow_rate"""

        self.flow_controller.set_flow_rate(request.goal_flowrate)

        # give the alicat a short time to adjust to the new flowrate before
        # returning the flow status
        time.sleep(0.1)

        # ask device for it's status
        flow_status = self.flow_controller.get()
        response.measured_flowrate = flow_status["mass_flow"]
        response.measured_pressure = flow_status["pressure"]
        response.measured_temperature = flow_status["temperature"]
        return response

    @contextmanager
    def exit_node_if_unable_to_open(self):
        """Log error and exit when an error occurs while opening an alicat_device"""

        try:
            # opened device without errors
            yield

        except:
            # error occured while opening, log an error and exit
            if self.device_serial_number:
                self.get_logger().error(
                    f"Failed opening Alicat device with serial number: {self.device_serial_number}. Device is not found or it is already opened."
                )
            elif self.port_from_param:
                self.get_logger().error(
                    f"Failed opening Alicat device with port: {self.port_from_param}. Device is not found or it is already opened."
                )
            else:
                self.get_logger().error(
                    f"Failed opening Alicat device. No free device is not found."
                )

            self.get_logger().error(f"Closing alicat node! ")

            exit()


def main():
    rclpy.init()
    flow_controller_node = AlicatNode()
    try:
        rclpy.spin(flow_controller_node)
    except KeyboardInterrupt:
        pass
    finally:
        # flow_controller_node.on_shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
