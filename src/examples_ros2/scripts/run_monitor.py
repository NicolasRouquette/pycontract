#!/usr/bin/env python3
import pycontract
import rclpy
from rclpy.node import Node
from examples_ros2.msg import A, B, C, D, E, F, End
from typing import Any

# Assume the OrMonitor and contract logic is similar to examples/run_monitor.py, but events are now ROS2 messages

class ROS2RunMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__('ros2_run_monitor')
        self.monitor = OrMonitor(self.get_logger())
        # Subscribe to all six event topics
        self.a_sub = self.create_subscription(A, 'event_a', self.a_callback, 10)
        self.b_sub = self.create_subscription(B, 'event_b', self.b_callback, 10)
        self.c_sub = self.create_subscription(C, 'event_c', self.c_callback, 10)
        self.d_sub = self.create_subscription(D, 'event_d', self.d_callback, 10)
        self.e_sub = self.create_subscription(E, 'event_e', self.e_callback, 10)
        self.f_sub = self.create_subscription(F, 'event_f', self.f_callback, 10)
        self.end_sub = self.create_subscription(End, 'end', self.end_callback, 10)

    def end_callback(self, msg: End) -> None:
        event = msg
        self.get_logger().info(f"Received End: {event}")
        self.monitor.end()
        rclpy.shutdown()

    def a_callback(self, msg: A) -> None:
        event = msg
        self.get_logger().info(f"Received A: {event}")
        self.monitor.eval(event)

    def b_callback(self, msg: B) -> None:
        event = msg
        self.get_logger().info(f"Received B: {event}")
        self.monitor.eval(event)

    def c_callback(self, msg: C) -> None:
        event = msg
        self.get_logger().info(f"Received C: {event}")
        self.monitor.eval(event)

    def d_callback(self, msg: D) -> None:
        event = msg
        self.get_logger().info(f"Received D: {event}")
        self.monitor.eval(event)

    def e_callback(self, msg: E) -> None:
        event = msg
        self.get_logger().info(f"Received E: {event}")
        self.monitor.eval(event)

    def f_callback(self, msg: F) -> None:
        event = msg
        self.get_logger().info(f"Received F: {event}")
        self.monitor.eval(event)

class OrMonitor(pycontract.Monitor):
    def __init__(self, logger):
        super().__init__()
        self.logger = logger

    """Example monitor using ROS2 messages as events."""
    def transition(self, event: Any) -> object:
        # You can use isinstance(event, A) etc. to branch
        if isinstance(event, A):
            # Handle A event
            self.logger.info(f"Processing A: {event}")
            return pycontract.OrState(
                OrMonitor.Expect_B_NotD_C(self.logger, event.x),
                OrMonitor.Expect_D_NotF_E(self.logger, event.x)
            )
        if isinstance(event, B):
            # Handle B event
            self.logger.info(f"Processing B: {event}")
            return pycontract.ok
        if isinstance(event, C):
            # Handle C event
            self.logger.info(f"Processing C: {event}")
            return pycontract.ok
        if isinstance(event, D):
            # Handle D event
            self.logger.info(f"Processing D: {event}")
            return ok
        if isinstance(event, E):
            # Handle E event
            self.logger.info(f"Processing E: {event}")
            return pycontract.ok
        if isinstance(event, F):
            # Handle F event
            self.logger.info(f"Processing F: {event}")
            return pycontract.ok
        return pycontract.error('Unknown event type')

    @pycontract.data
    class Expect_B_NotD_C(pycontract.HotState):
        x: int

        def __init__(self, logger, x):
            super().__init__()
            self.logger = logger
            self.x = x

        def transition(self, event: Any):
            if isinstance(event, B) and event.x == self.x:
                self.logger.info(f"Processing B: {event}")
                return OrMonitor.Expect_NotD_C(self.logger, self.x)
            return pycontract.error(f"Event {event} is not allowed in this branch")

    @pycontract.data
    class Expect_NotD_C(pycontract.HotState):
        x: int

        def __init__(self, logger, x):
            super().__init__()
            self.logger = logger
            self.x = x

        def transition(self, event):
            if isinstance(event, D) and event.x == self.x:
                self.logger.info(f"Processing D: {event}")
                return pycontract.error("D is not allowed in this branch")
            if isinstance(event, C) and event.x == self.x:
                self.logger.info(f"Processing C: {event}")
                return pycontract.ok

    @pycontract.data
    class Expect_D_NotF_E(pycontract.HotState):
        x: int

        def __init__(self, logger, x):
            super().__init__()
            self.logger = logger
            self.x = x

        def transition(self, event: Any):
            if isinstance(event, D) and event.x == self.x:
                self.logger.info(f"Processing D: {event}")
                return OrMonitor.Expect_NotF_E(self.logger, self.x)
            return pycontract.error(f"Event {event} is not allowed in this branch")

    @pycontract.data
    class Expect_NotF_E(pycontract.HotState):
        x: int

        def __init__(self, logger, x):
            super().__init__()
            self.logger = logger
            self.x = x

        def transition(self, event: Any):
            if isinstance(event, F) and event.x == self.x:
                self.logger.info(f"Processing F: {event}")
                return pycontract.error("F is not allowed in this branch")
            if isinstance(event, E) and event.x == self.x:
                self.logger.info(f"Processing E: {event}")
                return pycontract.ok
            return pycontract.error(f"Event {event} is not allowed in this branch")

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
