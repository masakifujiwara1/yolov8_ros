#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from yolov8_msgs.msg import DetectionArray

class LaserListener(Node):
    def __init__(self):
        super().__init__('laser_listener')
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/velodyne_2dscan_high_beams',
            self.callback,
            10)
        self.detections_sub = self.create_subscription(
            DetectionArray,
            '/yolo/tracking',
            self.callback_yolo,
            10)
        self.list = []
        self.scan = LaserScan()

    def callback_yolo(self, data):
        self.list = []
        for i in data.detections:
            if i.class_id == 0:
                self.list.append(i.bbox.center.theta)

        # angle = 3.14  # 取得したい角度（ラジアン）
        for angle in self.list:
            if angle < self.scan.angle_min or angle > self.scan.angle_max:
                self.get_logger().warn('指定した角度が範囲外です')
                return

            index = int((angle - self.scan.angle_min) / self.scan.angle_increment)

            if 0 <= index < len(self.scan.ranges):
                distance = self.scan.ranges[index]
                if distance == float('inf') and 0 <= index-1:
                    distance = self.scan.ranges[index-1]
                if distance == float('inf') and index+1 <= len(self.scan.ranges):
                    distance = self.scan.ranges[index+1]
                
                x, y = self.calc_xy(angle, distance)
                self.get_logger().info(f'Angle(rad): {angle}, Index: {index}, Distance(m): {distance}, x: {x}, y: {y}')
            else:
                self.get_logger().warn('計算されたインデックスが範囲外です')
        print("-" * 200)

    def callback(self, scan):
        self.scan = scan

    def calc_xy(self, angle, distance):
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)
        return x, y

def main(args=None):
    rclpy.init(args=args)
    laser_listener = LaserListener()
    rclpy.spin(laser_listener)
    laser_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
