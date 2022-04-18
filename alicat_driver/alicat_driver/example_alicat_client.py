#!/usr/bin/env python3

"""example_alicat_client
parameters: 
- flowrate_topic: name of topic to publish to 
- flowrate_service: name of service to call 
"""

import rclpy
from rclpy.node import Node
import time
import numpy as np
from alicat_driver_interfaces.srv import SetFlowRate


class FlowrateClient(Node):
    def __init__(self):
        super().__init__("airflow_test_client")

        default_param = {
            "flowrate_service": "set_flow_rate",
        }

        for key, value in default_param.items():
            if not self.has_parameter(key):
                self.declare_parameter(key, value)

        flow_rate_service = self.get_parameter("flowrate_service").value
        self.flow_rate_client = self.create_client(SetFlowRate, flow_rate_service)

        # if service cannot be found, close this node after 10 or so seconds
        counter = 0
        while not self.flow_rate_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("flow rate service not available, waiting..")
            counter += 1
            if counter >= 3:
                self.get_logger().error(
                    f"Could not connect to service: {flow_rate_service}, closing node"
                )
                exit()

        self.create_timer(1.0, self.timer_callback)
        self.flow_rate = 0.0

    def set_flow_rate(self, goal_flow_rate=0.5):
        """Call the flowrate service with the goal_flow_rate"""
        request = SetFlowRate.Request()
        request.goal_flow_rate = goal_flow_rate
        self.future = self.flow_rate_client.call_async(request)
        return self.future

    def timer_callback(self):
        self.set_flow_rate(goal_flow_rate=self.flow_rate)

        # increment flow rate, modulo 1
        self.flow_rate += 0.1
        self.flow_rate = self.flow_rate % 1.0


def main(args=None):
    rclpy.init(args=args)

    # flow_controller_node = FlowControllerNode()
    minimal_client = FlowrateClient()
    try:
        rclpy.spin(minimal_client)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
