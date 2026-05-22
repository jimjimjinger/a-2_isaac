#!/usr/bin/env python3

import rclpy

from isaac_localization.terrain_map_publisher import TerrainMapPublisher


class LocalizationNode(TerrainMapPublisher):
    def __init__(self):
        super().__init__()
        self.get_logger().info("Localization node is publishing map and estimated rover state.")


def main(args=None):
    rclpy.init(args=args)

    node = LocalizationNode()

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
