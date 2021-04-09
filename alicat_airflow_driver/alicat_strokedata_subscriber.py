#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from alicat import FlowController
from strokeflow_interfaces.msg import StrokeData
import time


class FlowControllerNode(Node):
    def __init__(self):
        super().__init__("flow_controller_node")

        self.declare_parameter("usb_port", "/dev/ttyUSB0")
        port = self.get_parameter("usb_port").value

        self.max_flow_rate = 1.0  # depends on alicat device
        self.mean_wba_sum = 130 * 2
        self.gain = 0.02
        self.mean_flow_rate = self.max_flow_rate / 2

        self.flow_controller = FlowController(port)

        strokedata_topic = "/stroke_analyzer/strokedata"
        self.sub_strokedata = self.create_subscription(
            StrokeData, strokedata_topic, self.strokedata_callback, 1
        )

    def strokedata_callback(self, strokedata_msg):
        # self.get_logger().info(f"strokedata: {strokedata_msg}")

        wba_sum = (
            strokedata_msg.wingbeat_amplitude_left
            + strokedata_msg.wingbeat_amplitude_right
        )

        wba_diff = wba_sum - self.mean_wba_sum

        flow_rate_setpoint = self.mean_flow_rate + wba_diff * self.gain

        if flow_rate_setpoint > (self.max_flow_rate - 0.1):
            flow_rate_setpoint = self.max_flow_rate
        if flow_rate_setpoint < 0.1:
            flow_rate_setpoint = 0.1

        self.flow_controller.set_flow_rate(flow_rate_setpoint)
        self.get_logger().info(
            f"strokedata: {wba_diff}, setpoint: {flow_rate_setpoint}"
        )
        # time.sleep(0.4)
        # flow_status = self.flow_controller.get()

        # response.measured_flow_rate = flow_status["volumetric_flow"]
        # response.measured_pressure = flow_status["volumetric_flow"]
        # response.measured_temperature = flow_status["temperature"]
        # self.get_logger().info(f"response at server: {response}")
        # return response


def main():
    rclpy.init()
    flow_controller_node = FlowControllerNode()
    rclpy.spin(flow_controller_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
