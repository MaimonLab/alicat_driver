#!/usr/bin/env python3

"""example_alicat_client
parameters: 
- flowrate_topic: name of topic to publish to 
- flowrate_service: name of service to call 
"""

import rclpy
from rclpy.node import Node
from alicat_driver_interfaces.msg import FlowRate


class FlowratePublisher(Node):
    def __init__(self):
        super().__init__("flowrate_publisher")

        default_param = {
            "goal_flowrate_topic": "alicat/goal_flow_rate",
        }

        for key, value in default_param.items():
            if not self.has_parameter(key):
                self.declare_parameter(key, value)

        flowrate_topic = self.get_parameter("goal_flowrate_topic").value
        self.pub = self.create_publisher(FlowRate, flowrate_topic, 1)

        self.create_timer(1.0, self.publish_flow_rate)
        self.flowrate = 1.0

    def publish_flow_rate(self):
        """Publish to the flowrate topic"""
        flowrate_msg = FlowRate()
        flowrate_msg.header.stamp = self.get_clock().now().to_msg()

        flowrate_msg.flow_rate = self.flowrate
        self.pub.publish(flowrate_msg)

        self.flowrate -= 0.1
        self.flowrate = self.flowrate % 1.0


def main(args=None):
    rclpy.init(args=args)

    minimal_client = FlowratePublisher()
    try:
        rclpy.spin(minimal_client)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
