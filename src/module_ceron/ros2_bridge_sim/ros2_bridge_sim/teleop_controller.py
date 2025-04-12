import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class MotorControllerNode(Node):
    def __init__(self):
        super().__init__("motor_controller_node")
        self.subscription = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        # Parámetros del robot
        wheel_base = 0.5  # distancia entre ruedas [m]

        # Cálculo de velocidad para motores diferencial
        v_left = linear - angular * wheel_base / 2.0
        v_right = linear + angular * wheel_base / 2.0

        self.get_logger().info(
            f"Motor Izquierdo: {v_left:.2f} m/s, Motor Derecho: {v_right:.2f} m/s"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
