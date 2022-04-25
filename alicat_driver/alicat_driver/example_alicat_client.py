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
            "flowrate_service": "set_flowrate",
        }

        for key, value in default_param.items():
            if not self.has_parameter(key):
                self.declare_parameter(key, value)

        flowrate_service = self.get_parameter("flowrate_service").value
        self.flowrate_client = self.create_client(SetFlowRate, flowrate_service)

        # if service cannot be found, close this node after 10 or so seconds
        counter = 0
        while not self.flowrate_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("flow rate service not available, waiting..")
            counter += 1
            if counter >= 3:
                self.get_logger().error(
                    f"Could not connect to service: {flowrate_service}, closing node"
                )
                exit()

        self.create_timer(1.0, self.timer_callback)
        self.flowrate = 0.0

    def set_flowrate(self, goal_flowrate=0.5):
        """Call the flowrate service with the goal_flowrate"""
        request = SetFlowRate.Request()
        request.goal_flowrate = goal_flowrate
        self.future = self.flowrate_client.call_async(request)
        return self.future

    def timer_callback(self):
        self.set_flowrate(goal_flowrate=self.flowrate)

        # increment flow rate, modulo 1
        self.flowrate += 0.1
        self.flowrate = self.flowrate % 1.0


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
