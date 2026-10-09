#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ForwardKinematics(Node):
    def __init__(self):
        super().__init__('forward_kinematics')

        self.r = 0.03
        self.L = 0.17
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev = None       # (sudut kiri, sudut kanan)
        self.prev_t = None     # waktu pesan sebelumnya (detik)

        self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)

    def joint_callback(self, msg):
        left = msg.position[msg.name.index('base_left_wheel_joint')]
        right = msg.position[msg.name.index('base_right_wheel_joint')]
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.prev is None:
            self.prev = (left, right)
            self.prev_t = t
            return

        dt = t - self.prev_t
        if dt <= 0.0:
            return

        d_left = self.r * (left - self.prev[0])
        d_right = self.r * (right - self.prev[1])
        self.prev = (left, right)
        self.prev_t = t

        ds = (d_right + d_left) / 2
        dtheta = (d_right - d_left) / self.L

        # kecepatan robot (sama satuannya dengan input inverse)
        v = ds / dt
        w = dtheta / dt

        # posisi (odometri)
        self.x += ds * math.cos(self.theta + dtheta / 2)
        self.y += ds * math.sin(self.theta + dtheta / 2)
        self.theta = math.atan2(math.sin(self.theta + dtheta),
                                math.cos(self.theta + dtheta))

        self.get_logger().info(
            f'v={v:.3f} m/s, w={w:.3f} rad/s | '
            f'x={self.x:.3f}, y={self.y:.3f}, theta={math.degrees(self.theta):.1f} deg',
            throttle_duration_sec=0.2)


def main():
    rclpy.init()
    node = ForwardKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()