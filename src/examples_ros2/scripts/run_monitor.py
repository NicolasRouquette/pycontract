#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pycontract import *
from examples_ros2.msg import A, B, C, D, E, F
from typing import Any

# Assume the OrMonitor and contract logic is similar to examples/run_monitor.py, but events are now ROS2 messages

class ROS2RunMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__('ros2_run_monitor')
        self.monitor = OrMonitor()
        # Subscribe to all six event topics
        self.a_sub = self.create_subscription(A, 'event_a', self.a_callback, 10)
        self.b_sub = self.create_subscription(B, 'event_b', self.b_callback, 10)
        self.c_sub = self.create_subscription(C, 'event_c', self.c_callback, 10)
        self.d_sub = self.create_subscription(D, 'event_d', self.d_callback, 10)
        self.e_sub = self.create_subscription(E, 'event_e', self.e_callback, 10)
        self.f_sub = self.create_subscription(F, 'event_f', self.f_callback, 10)

    def a_callback(self, msg: A) -> None:
        event = msg
        print(f"Received A: {event}")
        self.monitor.eval(event)

    def b_callback(self, msg: B) -> None:
        event = msg
        print(f"Received B: {event}")
        self.monitor.eval(event)

    def c_callback(self, msg: C) -> None:
        event = msg
        print(f"Received C: {event}")
        self.monitor.eval(event)

    def d_callback(self, msg: D) -> None:
        event = msg
        print(f"Received D: {event}")
        self.monitor.eval(event)

    def e_callback(self, msg: E) -> None:
        event = msg
        print(f"Received E: {event}")
        self.monitor.eval(event)

    def f_callback(self, msg: F) -> None:
        event = msg
        print(f"Received F: {event}")
        self.monitor.eval(event)

class OrMonitor(Monitor):
    """Example monitor using ROS2 messages as events."""
    def transition(self, event: Any) -> object:
        # You can use isinstance(event, A) etc. to branch
        if isinstance(event, A):
            # Handle A event
            print(f"Processing A: {event}")
            return OrState(
                OrMonitor.Expect_B_NotD_C(event.x),
                OrMonitor.Expect_D_NotF_E(event.x)
            )
        if isinstance(event, B):
            # Handle B event
            print(f"Processing B: {event}")
            return ok
        if isinstance(event, C):
            # Handle C event
            print(f"Processing C: {event}")
            return ok
        if isinstance(event, D):
            # Handle D event
            print(f"Processing D: {event}")
            return ok
        if isinstance(event, E):
            # Handle E event
            print(f"Processing E: {event}")
            return ok
        if isinstance(event, F):
            # Handle F event
            print(f"Processing F: {event}")
            return ok
        return error('Unknown event type')

    @data
    class Expect_B_NotD_C(HotState):
        x: int

        def transition(self, event: Any):
            if isinstance(event, B) and event.x == self.x:
                return OrMonitor.Expect_NotD_C(self.x)
            return error(f"Event {event} is not allowed in this branch")

    @data
    class Expect_NotD_C(HotState):
        x: int

        def transition(self, event):
            if isinstance(event, D) and event.x == self.x:
                return error("D is not allowed in this branch")
            if isinstance(event, C) and event.x == self.x:
                return ok

    @data
    class Expect_D_NotF_E(HotState):
        x: int

        def transition(self, event: Any):
            if isinstance(event, D) and event.x == self.x:
                return OrMonitor.Expect_NotF_E(self.x)
            return error(f"Event {event} is not allowed in this branch")

    @data
    class Expect_NotF_E(HotState):
        x: int

        def transition(self, event: Any):
            if isinstance(event, F) and event.x == self.x:
                return error("F is not allowed in this branch")
            if isinstance(event, E) and event.x == self.x:
                return ok
            return error(f"Event {event} is not allowed in this branch")

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
