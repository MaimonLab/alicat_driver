#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from alicat import FlowController
from alicat_driver_interfaces.srv import SetFlowRate
from alicat_driver_interfaces.msg import FlowRate
import time
import serial.tools.list_ports
from typing import Optional


def find_port_for_serial(serial_id: str) -> Optional[str]:
    """Returns port string if found, otherwise returns None"""

    ports = list(serial.tools.list_ports.comports())

    for port in ports:
        if port.serial_number == serial_id:
            return port.device

    return None


class FlowControllerNode(Node):
    def __init__(self):
        super().__init__("flow_controller_node")
        default_param = {
            "serial_port": None,
            "device_serial_number": None,
            "subscribe_flowrate_topic": True,
            "flowrate_topic": "alicat/flow_rate",
        }

        for key, value in default_param.items():
            if not self.has_parameter(key):
                self.declare_parameter(key, value)

        port_from_param = self.get_parameter("serial_port").value
        device_serial_number = self.get_parameter("device_serial_number").value

        # throw error if Serial id consists only of numbers and is not specified as string
        if not (type(device_serial_number) is str) and (
            device_serial_number is not None
        ):
            raise TypeError(
                f"device_serial_number should be of type string! It is currently type: {type(device_serial_number)}"
            )

        # if a serial number is specified, get a port to try and open
        if device_serial_number is not None:
            port_to_try = find_port_for_serial(device_serial_number)

            if port_to_try is None:
                self.get_logger().error(
                    f"No serial port found for serial number {device_serial_number}, using mock serial"
                )
            elif port_to_try is not None:
                self.get_logger().info(
                    f"device_serial_number specified in config: {device_serial_number}"
                )
        elif port_from_param is not None:
            port_to_try = port_from_param
        else:
            port_to_try = "not specified"

        # if a port is specified, try to open it. Open a mock part in case of faillure
        if port_to_try != "not specified":
            try:
                # self.rig_controller = ArduinoInterface(port_to_try)
                self.flow_controller = FlowController(port_to_try)
                if port_to_try is not None:
                    self.get_logger().info(f"Opening Serial Port {port_to_try}")
            except serial.SerialException as e:
                self.get_logger().error(
                    f"Error opening serial port {port_to_try}. Using mock serial"
                )
                self.flow_controller = FlowController()
        else:
            self.get_logger().error(
                f"No port or serial number specified. Using mock serial"
            )
            self.flow_controller = FlowController()

        self.create_service(SetFlowRate, "set_flow_rate", self.set_flow_rate_callback)

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
        # self.get_logger().info(f"response at server: {response}")
        return response


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
