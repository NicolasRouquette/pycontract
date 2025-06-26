#!/usr/bin/env python3
import pycontract
import rclpy
from dataclasses import field
from rclpy.node import Node

from examples_ros2.msg import A, B, C, D, E, F, End
from typing import Any

from pycontract_ros2 import (
    enable_pattern_matching_for_ros2_package,
)

# Assume the OrMonitor and contract logic is similar to examples/run_monitor.py, but events are now ROS2 messages

class ROS2RunMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__('ros2_run_monitor')
        enable_pattern_matching_for_ros2_package('examples_ros2')
        self.monitor = OrMonitor(self.get_logger())

        topic_types = {
            'event_a': A,
            'event_b': B,
            'event_c': C,
            'event_d': D,
            'event_e': E,
            'event_f': F,
        }

        for name, msg_type in topic_types.items():
            self.create_subscription(msg_type, name, self._event_cb, 10)

        self.create_subscription(End, 'end', self._end_cb, 10)

    def _event_cb(self, msg) -> None:
        # A|B|C|D|E|F all reach here
        self.get_logger().info(f"Received {msg.__class__.__name__}: {msg}")
        self.monitor.eval(msg)

    def _end_cb(self, _: End) -> None:
        self.get_logger().info(f"Received End")
        self.monitor.end()
        rclpy.shutdown()

class OrMonitor(pycontract.Monitor):
    def __init__(self, logger):
        super().__init__()
        self.logger = logger

    """Example monitor using ROS2 messages as events."""
    
    def transition(self, event):
        match event:
            case A(x):
                self.logger.info(f"Processing A: {event}")
                return pycontract.OrState(
                    OrMonitor.Expect_B_NotD_C(self.logger, x),
                    OrMonitor.Expect_D_NotF_E(self.logger, x)
                )

    @pycontract.data
    class Expect_B_NotD_C(pycontract.HotState):
        logger: Any = field(repr=False, compare=False)
        x: int

        def transition(self, event):
            match event:
                case B(self.x):
                    self.logger.info(f"Processing B: {event}")
                    return OrMonitor.Expect_NotD_C(self.logger, self.x)

    @pycontract.data
    class Expect_NotD_C(pycontract.HotState):
        logger: Any = field(repr=False, compare=False)
        x: int

        def transition(self, event):
            match event:
                case D(self.x):
                    self.logger.info(f"Processing D: {event}")
                    return pycontract.error("D is not allowed in this branch")
                case C(self.x):
                    self.logger.info(f"Processing C: {event}")
                    return pycontract.ok

    @pycontract.data
    class Expect_D_NotF_E(pycontract.HotState):
        logger: Any = field(repr=False, compare=False)
        x: int

        def transition(self, event: Any):
            match event:
                case D(self.x):
                    self.logger.info(f"Processing D: {event}")
                    return OrMonitor.Expect_NotF_E(self.logger, self.x)

    @pycontract.data
    class Expect_NotF_E(pycontract.HotState):
        logger: Any = field(repr=False, compare=False)
        x: int

        def transition(self, event: Any):
            match event:
                case F(self.x):
                    self.logger.info(f"Processing F: {event}")
                    return pycontract.error("F is not allowed in this branch")
                case E(self.x):
                    self.logger.info(f"Processing E: {event}")
                    return pycontract.ok

def main() -> None:
    rclpy.init()
    node = ROS2RunMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.monitor.end()

if __name__ == '__main__':
    main()
