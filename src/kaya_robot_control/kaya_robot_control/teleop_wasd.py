#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame
import time

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Init pygame for keyboard input
        pygame.init()
        self.screen = pygame.display.set_mode((100, 100))  # Dummy window
        self.timer_period = 0.05  # 20 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.linear_speed = 0.2
        self.vx = 0.0
        self.vy = 0.0

    def timer_callback(self):
        pygame.event.pump()  # Process event queue
        keys = pygame.key.get_pressed()

        # Reset velocities
        self.vx, self.vy = 0.0, 0.0

        # WASD control
        if keys[pygame.K_w]:
            self.vx = self.linear_speed
        elif keys[pygame.K_s]:
            self.vx = -self.linear_speed
        if keys[pygame.K_a]:
            self.vy = self.linear_speed
        elif keys[pygame.K_d]:
            self.vy = -self.linear_speed

        twist = Twist()
        twist.linear.x = self.vx
        twist.linear.y = self.vy
        twist.angular.z = 0.0

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    teleop_node = KeyboardTeleop()
    try:
        rclpy.spin(teleop_node)
    except KeyboardInterrupt:
        pass
    finally:
        teleop_node.destroy_node()
        rclpy.shutdown()
        pygame.quit()

if __name__ == '__main__':
    main()