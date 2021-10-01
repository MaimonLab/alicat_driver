#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import numpy as np


# from strokeflow_interfaces.srv import SetFlowRate
from alicat_driver_interfaces.srv import SetFlowRate
from alicat_driver_interfaces.msg import FlowRate

from typing import Optional


class AirflowTestClient(Node):
    def __init__(self):
        super().__init__("airflow_test_client")

        default_param = {
            "flowrate_topic": "alicat/flow_rate",
            "flowrate_service": "set_flow_rate",
        }

        # breakpoint()
        for key, value in default_param.items():
            if not self.has_parameter(key):
                self.declare_parameter(key, value)

        flow_rate_service = self.get_parameter("flowrate_service").value
        self.flow_rate_client = self.create_client(SetFlowRate, flow_rate_service)

        while not self.flow_rate_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("flow rate service not available, waiting..")

        flowrate_topic = self.get_parameter("flowrate_topic").value
        self.pub = self.create_publisher(FlowRate, flowrate_topic, 1)

    def set_flow_rate(self, goal_flow_rate=0.5):
        request = SetFlowRate.Request()
        request.goal_flow_rate = goal_flow_rate
        self.future = self.flow_rate_client.call_async(request)
        return self.future

    def publish_flow_rate(self, goal_flow_rate=0.5):
        flowrate_msg = FlowRate()
        flowrate_msg.flow_rate = goal_flow_rate
        self.pub.publish(flowrate_msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = AirflowTestClient()

    # while True:
    # for flow_rate in [0.3, 0.6, 0.8]:
    for flow_rate in np.arange(1, 0, -0.1):
        minimal_client.set_flow_rate(goal_flow_rate=flow_rate)
        time.sleep(1)

    # for flow_rate in [0.3, 0.6, 0.8]:
    # while True:
    for flow_rate in np.arange(0, 1, 0.1):
        minimal_client.publish_flow_rate(goal_flow_rate=flow_rate)
        time.sleep(1)

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
