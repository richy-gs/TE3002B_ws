import rclpy
from rclpy.node import Node


class PointStabilisationNode(Node):
    def __init__(self):
        return

        # WRITE YOUR CODE HERE


def main(args=None):
    rclpy.init(args=args)

    node = PointStabilisationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():  # Ensure shutdown is only called once
            rclpy.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
