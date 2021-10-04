#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from alicat import FlowController
from alicat_driver_interfaces.srv import SetFlowRate
from alicat_driver_interfaces.msg import FlowRate
import time
import serial.tools.list_ports
from typing import Optional
from contextlib import contextmanager


def find_port_for_serial(serial_id: str) -> Optional[str]:
    """Returns port string if found, otherwise returns None"""

    ports = list(serial.tools.list_ports.comports())

    for port in ports:
        if port.serial_number == serial_id:
            return port.device

    return None


def get_alicat_port():

    ports = list(serial.tools.list_ports.comports())

    for port in ports:
        if port.manufacturer == "FTDI":
            return port.device
    return None


def find_alicat_serial_from_port(flow_controller):

    flow_controller_data = flow_controller.open_ports
    for key, item in flow_controller_data.items():

        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            if port.device == item[0].port:
                found_serial_number = port.serial_number
                return found_serial_number
    return None


class FlowControllerNode(Node):
    def __init__(self):
        super().__init__("flow_controller_node")
        default_param = {
            "device_port": None,
            "device_serial_number": None,
            "subscribe_flowrate_topic": True,
            "flowrate_topic": "alicat/flow_rate",
            "flowrate_service": "set_flow_rate",
        }

        # breakpoint()
        for key, value in default_param.items():
            if not self.has_parameter(key):
                self.declare_parameter(key, value)

        self.port_from_param = self.get_parameter("device_port").value
        self.device_serial_number = self.get_parameter("device_serial_number").value

        if self.device_serial_number:
            self.device_serial_number = self.device_serial_number.replace("$", "")

        with self.log_error_if_unavailable():
            # if True:
            if self.device_serial_number:
                port_to_try = find_port_for_serial(self.device_serial_number)
                if port_to_try is None:
                    raise FileNotFoundError(b"Cannot find port for serial")
                self.flow_controller = FlowController(port_to_try)
                self.get_logger().info(
                    f"Opened Alicat w s/n: {self.device_serial_number} on {port_to_try}"
                )
                # self.get_logger().info(f"On port: {port_to_try}")
            elif self.port_from_param:
                self.get_logger().info(f"Port from param: {self.port_from_param}")
                self.flow_controller = FlowController(self.port_from_param)
                found_serial_number = find_alicat_serial_from_port(self.flow_controller)
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

        self.create_service(SetFlowRate, flowrate_service, self.set_flow_rate_callback)

        if self.get_parameter("subscribe_flowrate_topic").value:
            flowrate_topic = self.get_parameter("flowrate_topic").value
            self.create_subscription(
                FlowRate, flowrate_topic, self.flowrate_callback, 1
            )

    def flowrate_callback(self, flowrate_msg):

        raw_flowrate = flowrate_msg.flow_rate

        if raw_flowrate < 0:
            apply_flowrate = 0
        else:
            apply_flowrate = raw_flowrate

        self.flow_controller.set_flow_rate(apply_flowrate)

    def set_flow_rate_callback(self, request, response):

        self.flow_controller.set_flow_rate(request.goal_flow_rate)
        time.sleep(0.4)
        flow_status = self.flow_controller.get()

        response.measured_flow_rate = flow_status["volumetric_flow"]
        response.measured_pressure = flow_status["volumetric_flow"]
        response.measured_temperature = flow_status["temperature"]
        return response

    @contextmanager
    def log_error_if_unavailable(self):
        """Log error when an error occurs while opening an mcc device either with or without serial number"""
        try:
            yield
        except:
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
    flow_controller_node = FlowControllerNode()
    try:
        rclpy.spin(flow_controller_node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
