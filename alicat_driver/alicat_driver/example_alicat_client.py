#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time


# from strokeflow_interfaces.srv import SetFlowRate
from alicat_driver_interfaces.srv import SetFlowRate

from typing import Optional


class AirflowTestClient(Node):
    def __init__(self):
        super().__init__("airflow_test_client")

        # see if config_found parameter is found in yaml file

        self.flow_rate_client = self.create_client(SetFlowRate, "/set_flow_rate")
        while not self.flow_rate_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("flow rate service not available, waiting..")

    def set_flow_rate(self, goal_flow_rate=0.5):
        request = SetFlowRate.Request()
        request.goal_flow_rate = goal_flow_rate
        self.get_logger().info(f"Sending flow rate request {request.goal_flow_rate}")
        self.future = self.flow_rate_client.call_async(request)
        return self.future


def main(args=None):
    rclpy.init(args=args)

    minimal_client = AirflowTestClient()

    while True:
        for flow_rate in [0.3, 0.6, 0.8]:
            minimal_client.set_flow_rate(goal_flow_rate=flow_rate)
            time.sleep(5)

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
