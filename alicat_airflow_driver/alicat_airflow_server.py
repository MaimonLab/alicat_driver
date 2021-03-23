#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from alicat import FlowController
from strokeflow_interfaces.srv import SetFlowRate
import time


class FlowControllerNode(Node):
    def __init__(self):
        super().__init__("flow_controller_node")

        self.declare_parameter("usb_port", "/dev/ttyUSB1")
        port = self.get_parameter("usb_port").value

        self.flow_controller = FlowController(port)

        self.create_service(SetFlowRate, "set_flow_rate", self.set_flow_rate_callback)

    def set_flow_rate_callback(self, request, response):

        self.flow_controller.set_flow_rate(request.goal_flow_rate)
        time.sleep(0.4)
        flow_status = self.flow_controller.get()

        response.measured_flow_rate = flow_status["volumetric_flow"]
        response.measured_pressure = flow_status["volumetric_flow"]
        response.measured_temperature = flow_status["temperature"]
        self.get_logger().info(f"response at server: {response}")
        return response


def main():
    rclpy.init()
    flow_controller_node = FlowControllerNode()
    rclpy.spin(flow_controller_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
