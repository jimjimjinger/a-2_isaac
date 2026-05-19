"""Simulation service bridge for the Mars rover ROS2 module skeleton."""

from __future__ import annotations

import os

import rclpy
from isaac_interfaces.srv import CheckSystemReady, ResetSimulation, SaveExplorationMap
from rclpy.node import Node


class SimBridgeNode(Node):
    """Provides simulation lifecycle services.

    This node is the future boundary to Isaac Sim. For now it returns successful
    mock responses so mission orchestration can be tested independently.
    """

    def __init__(self) -> None:
        super().__init__("sim_bridge_node")
        self.declare_parameter("ready", True)
        self.declare_parameter("required_nodes", [])
        self.declare_parameter("default_map_dir", "/tmp")

        self.create_service(CheckSystemReady, "/check_system_ready", self._check_system_ready)
        self.create_service(ResetSimulation, "/reset_simulation", self._reset_simulation)
        self.create_service(SaveExplorationMap, "/save_exploration_map", self._save_exploration_map)
        self.get_logger().info("sim_bridge_node ready: lifecycle services available")

    def _check_system_ready(
        self,
        request: CheckSystemReady.Request,
        response: CheckSystemReady.Response,
    ) -> CheckSystemReady.Response:
        response.ready = bool(self.get_parameter("ready").value)
        response.missing_nodes = []
        response.message = f"ready for {request.requester}" if response.ready else "mock bridge not ready"
        return response

    def _reset_simulation(
        self,
        request: ResetSimulation.Request,
        response: ResetSimulation.Response,
    ) -> ResetSimulation.Response:
        response.success = True
        response.message = (
            "mock reset accepted "
            f"(world={request.reset_world}, robot={request.reset_robot}, "
            f"mission={request.reset_mission}, reason={request.reason})"
        )
        return response

    def _save_exploration_map(
        self,
        request: SaveExplorationMap.Request,
        response: SaveExplorationMap.Response,
    ) -> SaveExplorationMap.Response:
        target_dir = request.target_path or str(self.get_parameter("default_map_dir").value)
        response.success = True
        response.saved_path = os.path.join(target_dir, f"{request.map_name}.yaml")
        response.message = "mock map save completed"
        return response


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SimBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
