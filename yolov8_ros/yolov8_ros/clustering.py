import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
from sklearn.cluster import DBSCAN

class LaserScanSubscriber(Node):
    def __init__(self):
        super().__init__('laser_scan_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            'velodyne_2dscan_high_beams',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(LaserScan, 'detected_pedestrians', 10)
        self.subscription  # prevent unused variable warning
        self.scan = LaserScan()

    def detect_pedestrian(self, ranges):
        # スキャンデータを2次元座標に変換
        angle_min = self.scan.angle_min  # 仮の角度範囲
        angle_increment = self.scan.angle_increment
        points = []
        
        for i, range in enumerate(ranges):
            if np.isfinite(range):  # NaNや無限大をフィルタリング
                angle = angle_min + i * angle_increment
                x = range * np.cos(angle)
                y = range * np.sin(angle)
                points.append([x, y])
        
        points = np.array(points)

        if len(points) == 0:
            return  # 有効なデータがない場合は終了

        # DBSCANによるクラスタリング
        clustering = DBSCAN(eps=0.5, min_samples=5).fit(points)
        labels = clustering.labels_

        pedestrian_points = []
        # クラスタをフィルタリングして歩行者を特定
        unique_labels = set(labels)
        for label in unique_labels:
            if label == -1:
                continue
            class_member_mask = (labels == label)
            xy = points[class_member_mask]

            # クラスタのサイズをチェック
            if self.is_pedestrian_cluster(xy):
                pedestrian_points.extend(xy)

        return pedestrian_points
    
    def is_pedestrian_cluster(self, cluster):
        # クラスタのサイズや形状を基に歩行者かどうかを判定
        # ここでは簡単な例として、クラスタの点の数を基に判定
        # if 50 > len(cluster) > 3:  # 適当な閾値
        if len(cluster) > 5:  # 適当な閾値
            return True
        return False

    def listener_callback(self, msg):
        # レーザースキャンデータの処理をここに書く
        self.scan = msg
        pedestrian_points = self.detect_pedestrian(self.scan.ranges)
        if pedestrian_points is not None:
            self.publish_pedestrians(msg, pedestrian_points)

    def publish_pedestrians(self, original_msg, pedestrian_points):
        pedestrian_msg = LaserScan()
        pedestrian_msg.header = original_msg.header
        pedestrian_msg.angle_min = original_msg.angle_min
        pedestrian_msg.angle_max = original_msg.angle_max
        pedestrian_msg.angle_increment = original_msg.angle_increment
        pedestrian_msg.time_increment = original_msg.time_increment
        pedestrian_msg.scan_time = original_msg.scan_time
        pedestrian_msg.range_min = original_msg.range_min
        pedestrian_msg.range_max = original_msg.range_max

        # 全ての範囲データを初期化
        pedestrian_msg.ranges = [float('inf')] * len(original_msg.ranges)

        for point in pedestrian_points:
            angle = np.arctan2(point[1], point[0])
            if original_msg.angle_min <= angle <= original_msg.angle_max:
                index = int((angle - original_msg.angle_min) / original_msg.angle_increment)
                distance = np.sqrt(point[0]**2 + point[1]**2)
                pedestrian_msg.ranges[index] = distance

        self.publisher.publish(pedestrian_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
