#!/usr/bin/env python3
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class InverseKinematics(Node):
    def __init__(self):
        super().__init__('inverse_kinematics')

        self.wheel_radius = 0.03
        self.wheel_separation = 0.17

        self.create_subscription(Twist, '/input_ik', self.velocity_callback, 10)

        self.left_publisher = self.create_publisher(Float64, '/left_wheel/command', 10)
        self.right_publisher = self.create_publisher(Float64, '/right_wheel/command', 10)

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('Inverse kinematics aktif.')

        threading.Thread(target=self.input_loop, daemon=True).start()

    def velocity_callback(self, msg):
        self.hitung_dan_kirim(msg.linear.x, msg.angular.z)

    def hitung_dan_kirim(self, v, w):
        wl = (v - w * self.wheel_separation / 2) / self.wheel_radius
        wr = (v + w * self.wheel_separation / 2) / self.wheel_radius

        left = Float64()
        right = Float64()
        left.data = wl
        right.data = wr
        self.left_publisher.publish(left)
        self.right_publisher.publish(right)

        cmd = Twist()
        cmd.linear.x = self.wheel_radius * (wr + wl) / 2
        cmd.angular.z = self.wheel_radius * (wr - wl) / self.wheel_separation
        self.cmd_vel_publisher.publish(cmd)

    def input_loop(self):
        print('Masukkan: v w   (v = m/s, w = rad/s), contoh: 0.1 0.0')
        print('Ketik s untuk berhenti, Ctrl+C untuk keluar.')
        while rclpy.ok():
            try:
                text = input('v w> ').strip().lower()
            except EOFError:
                return
            if text == 's':
                self.hitung_dan_kirim(0.0, 0.0)
                print('Berhenti.')
                continue
            try:
                v, w = map(float, text.split())
            except ValueError:
                print('Format salah. Contoh: 0.1 0.0')
                continue
            self.hitung_dan_kirim(v, w)
            print(f'Terkirim: v={v} m/s, w={w} rad/s')


def main():
    rclpy.init()
    node = InverseKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.hitung_dan_kirim(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()