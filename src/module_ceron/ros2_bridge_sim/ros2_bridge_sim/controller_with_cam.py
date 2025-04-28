#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image


class BallFollower(Node):
    def __init__(self):
        super().__init__("ball_follower")
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, "/image", self.image_callback, 10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # PID angular
        self.kp_ang = 0.005
        self.ki_ang = 0.0001
        self.kd_ang = 0.001

        self.integral_ang = 0.0
        self.prev_error_ang = 0.0

        # PID lineal (para mantener distancia)
        self.kp_lin = 0.001
        self.ki_lin = 0.00005
        self.kd_lin = 0.0001

        self.integral_lin = 0.0
        self.prev_error_lin = 0.0

        self.target_area = (
            10000  # área de la pelota que representa la distancia deseada
        )
        self.max_linear = 0.2  # velocidad lineal máxima (m/s)
        self.max_angular = 1.0  # velocidad angular máxima (rad/s)

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Rango para pelota verde
        lower_green = np.array([40, 70, 70])
        upper_green = np.array([80, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Procesamiento de contornos
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        height, width, _ = img.shape
        twist = Twist()

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            area = cv2.contourArea(largest_contour)

            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])

                # ---------- Control angular ----------
                error_ang = cx - width // 2
                self.integral_ang += error_ang
                derivative_ang = error_ang - self.prev_error_ang
                angular_z = -(
                    self.kp_ang * error_ang
                    + self.ki_ang * self.integral_ang
                    + self.kd_ang * derivative_ang
                )
                self.prev_error_ang = error_ang

                # ---------- Control lineal (distancia) ----------
                error_lin = self.target_area - area  # positivo si está lejos
                self.integral_lin += error_lin
                derivative_lin = error_lin - self.prev_error_lin
                linear_x = (
                    self.kp_lin * error_lin
                    + self.ki_lin * self.integral_lin
                    + self.kd_lin * derivative_lin
                )
                self.prev_error_lin = error_lin

                # ---------- Saturación ----------
                twist.linear.x = float(
                    np.clip(linear_x, -self.max_linear, self.max_linear)
                )
                twist.angular.z = float(
                    np.clip(angular_z, -self.max_angular, self.max_angular)
                )

            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

        else:
            # Pelota no detectada → detener
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = BallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
