"""Shared plumbing of the Python nodes: required parameters and main()."""

from typing import Callable, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter


def required_parameter(node: Node, name: str, parameter_type: Parameter.Type):
    """Declare a parameter without a default and return its value; raise if it is not set."""
    node.declare_parameter(name, parameter_type)
    return node.get_parameter(name).value


def run_node(node_factory: Callable[[], Node],
             on_shutdown: Optional[Callable[[Node], None]] = None, args=None) -> None:
    """Init rclpy, spin the node until shutdown, clean up quietly."""
    rclpy.init(args=args)
    node = None
    try:
        node = node_factory()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl-C in a terminal reaches the node twice (from the TTY and from
        # launch); the second must not abort the cleanup.
        try:
            if node is not None:
                if on_shutdown is not None:
                    on_shutdown(node)
                node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()
