#!/usr/bin/env python3
import rclpy
import math
from collections import defaultdict
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from yolov8_msgs.msg import DetectionArray
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker, MarkerArray

class LaserListener(Node):
    def __init__(self):
        super().__init__('laser_listener')
        scan_qos = rclpy.qos.QoSProfile(depth=10)
        scan_qos.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
        self.scan_sub = self.create_subscription(
            LaserScan,
            # '/detected_pedestrians',
            # '/velodyne_2dscan_high_beams',
            '/scan',
            self.callback,
            scan_qos)
        self.detections_sub = self.create_subscription(
            DetectionArray,
            '/yolo/tracking',
            self.callback_yolo,
            10)
        self.marker_array_pub = self.create_publisher(MarkerArray, 'detect_human', 10)
        self.dicts = defaultdict(lambda: {'id': 0, 'score': 0, 'theta': 0, 'x': 0, 'y': 0, 'size_x': 0, 'size_y': 0})
        self.scan = LaserScan()
        self.marker_array = MarkerArray()

    def callback_yolo(self, data):
        self.dicts.clear()
        self.marker_array = MarkerArray()
        for idx, i in enumerate(data.detections):
            if i.class_id == 0:
                self.dicts[idx]['id'] = i.id
                self.dicts[idx]['score'] = i.score
                self.dicts[idx]['theta'] = i.bbox.center.theta
                self.dicts[idx]['size_x'] = i.bbox.size.x
                self.dicts[idx]['size_y'] = i.bbox.size.y

        for key, value in self.dicts.items():
            # print(key, value) key: 0 value: {'id': 1537 ...etc...}
            angle = value['theta']
            id_ = value['id']
            score = value['score']
            size_x = value['size_x']
            size_y = value['size_y']

            if angle < self.scan.angle_min or angle > self.scan.angle_max:
                self.get_logger().warn(f'指定した角度が範囲外です. angle={angle}, min={self.scan.angle_min}')
                return

            index = int((angle - self.scan.angle_min) / self.scan.angle_increment)

            if 0 <= index < len(self.scan.ranges):
                distance = self.scan.ranges[index]
                if distance == float('inf'):
                    for offset in range(-1, 1):
                        new_index = index + offset
                        if self.is_valid_index(new_index, len(self.scan.ranges)):
                                distance = self.scan.ranges[new_index]
                        if distance != float('inf'):
                            break
                
                x, y = self.calc_xy(angle, distance)
                # if score >= 0.75:
                # self.get_logger().info(f'Angle(rad): {angle}, Index: {index}, Distance(m): {distance}, x: {x}, y: {y}, tracking_id: {id_}')
                self.get_logger().info(f'Angle(rad): {angle}, Index: {index}, Distance(m): {distance}, tracking_id: {id_}, size_x: {size_x}, size_y: {size_y}')

                # if distance == float('inf'):
                #     self.get_logger().info(f'Angle(rad): {angle}, Index: {index}, tracking_id: {id_}, score: {score}, size_x: {size_x}, size_y: {size_y}, items: {len(self.dicts.items())}')

                self.dicts[key]['x'] = x
                self.dicts[key]['y'] = y

                if distance != float('inf'):
                    self.add_marker(id_, x, y)
            else:
                self.get_logger().warn('計算されたインデックスが範囲外です')

        self.publish_marker_array()

        print("-" * 120)

    def callback(self, scan):
        self.scan = scan
        # self.get_logger().info(f'received scan topic')

    def calc_xy(self, angle, distance):
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)
        return x, y

    def is_valid_index(self, i, length):
        return 0 <= i < length

    def add_marker(self, id_, x, y):
        point = Marker()
        point.header.frame_id = 'upper_velodyne_frame'
        point.ns = "point"
        point.id = int(id_)
        point.type = Marker.SPHERE
        point.action = Marker.ADD
        point.pose.position.x = x
        point.pose.position.y = y
        point.pose.position.z = 0.0
        point.scale.x = point.scale.y = point.scale.z = 1.0
        point.color.r = 0.0
        point.color.g = 1.0
        point.color.b = 0.0
        point.color.a = 1.0
        point.lifetime = rclpy.duration.Duration(seconds = 0.1).to_msg()
        self.marker_array.markers.append(point)
    
    def publish_marker_array(self):
        self.marker_array_pub.publish(self.marker_array)

def main(args=None):
    rclpy.init(args=args)
    laser_listener = LaserListener()
    rclpy.spin(laser_listener)
    laser_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
