#!/usr/bin/env python3
"""LDS-02 LiDAR driver: reads the serial protocol and publishes LaserScan."""

import math
import struct

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


PORT = "/dev/ttyUSB0"
BAUD = 115200
MIN_RANGE_MM = 150
MAX_RANGE_MM = 12000
SCAN_RESOLUTION = 0.5
CONFIDENCE_THRESHOLD = 100
PACKETS_PER_ROTATION = 30

# LDS-02 frame: 2-byte header (0x54 0x2C) + 45 byte payload = 47 bytes
PACKET_HEADER = b'\x54\x2c'
PACKET_SIZE = 47


class LDSDriverNode(Node):
    def __init__(self):
        super().__init__('lds_driver')

        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)

        try:
            self.ser = serial.Serial(PORT, BAUD, timeout=1)
            self.get_logger().info(f'LDS-02 Connected on {PORT}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to LiDAR: {e}')
            return

        self.buffer = bytearray()
        self.scan_data = {}
        self.packets_in_rotation = 0

        self.create_timer(0.001, self.read_serial_callback)

    def read_serial_callback(self):
        try:
            if self.ser.in_waiting > 0:
                self.buffer += self.ser.read(min(self.ser.in_waiting, 500))

            if len(self.buffer) > 2000:
                self.buffer = self.buffer[-1000:]

            while len(self.buffer) >= PACKET_SIZE:
                start = self.buffer.find(PACKET_HEADER)

                if start == -1:
                    self.buffer = self.buffer[-1:]
                    return

                if len(self.buffer) < start + PACKET_SIZE:
                    # Partial packet, wait for more
                    return

                packet = self.buffer[start:start + PACKET_SIZE]
                self.process_packet(packet)
                self.buffer = self.buffer[start + PACKET_SIZE:]

        except Exception as e:
            self.get_logger().error(f'Serial read error: {e}')

    def process_packet(self, packet):
        start_angle = struct.unpack('<H', packet[4:6])[0] / 100.0
        end_angle = struct.unpack('<H', packet[42:44])[0] / 100.0

        # Handle the 360 -> 0 wraparound inside a single packet
        if end_angle < start_angle:
            end_angle += 360.0

        step = (end_angle - start_angle) / 11.0

        for i in range(12):
            base = 6 + (i * 3)
            distance = struct.unpack('<H', packet[base:base + 2])[0]
            confidence = packet[base + 2]

            angle = start_angle + (i * step)
            if angle >= 360.0:
                angle -= 360.0

            if distance >= MIN_RANGE_MM and confidence > CONFIDENCE_THRESHOLD:
                self.scan_data[angle] = distance

        self.packets_in_rotation += 1

        if self.packets_in_rotation >= PACKETS_PER_ROTATION:
            self.publish_scan()
            self.scan_data = {}
            self.packets_in_rotation = 0

    def publish_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'

        scan.angle_min = 0.0
        scan.angle_max = 2.0 * math.pi
        scan.angle_increment = math.radians(SCAN_RESOLUTION)
        scan.time_increment = 0.0
        scan.scan_time = 0.15
        scan.range_min = MIN_RANGE_MM / 1000.0
        scan.range_max = MAX_RANGE_MM / 1000.0

        num_readings = int(360.0 / SCAN_RESOLUTION)
        ranges = [float('inf')] * num_readings

        for angle, dist_mm in self.scan_data.items():
            idx = int(angle / SCAN_RESOLUTION)
            if 0 <= idx < num_readings:
                ranges[idx] = dist_mm / 1000.0

        scan.ranges = ranges
        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = LDSDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
