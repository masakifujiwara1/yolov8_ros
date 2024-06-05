import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
from sklearn.cluster import DBSCAN
from scipy.optimize import linear_sum_assignment
from sklearn.metrics.pairwise import euclidean_distances

class LaserListener(Node):

    def __init__(self):
        super().__init__('laser_listener')
        self.subscription = self.create_subscription(
            LaserScan,
            '/velodyne_2dscan_high_beams',
            self.laser_callback,
            10)
        self.previous_clusters = None
        self.track_ids = {}
        self.current_time = None

    def laser_callback(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        points = np.array([ranges * np.cos(angles), ranges * np.sin(angles)]).T

        # 前処理：無限値やNaNの削除
        points = points[np.isfinite(points).all(axis=1)]

        # クラスタリング
        dbscan = DBSCAN(eps=0.2, min_samples=10)
        labels = dbscan.fit_predict(points)

        # クラスタの特徴量抽出
        clusters = self.extract_cluster_features(points, labels)

        # トラッキングの更新
        if self.previous_clusters is not None:
            self.track_ids = self.update_tracking(self.previous_clusters, clusters, self.track_ids)
        else:
            # 初期ID設定
            for i, label in enumerate(clusters.keys()):
                self.track_ids[label] = i

        self.previous_clusters = clusters
        self.current_time = msg.header.stamp

        print(len(self.track_ids.keys()))

    def extract_cluster_features(self, points, labels):
        clusters = {}
        for label in set(labels):
            if label == -1:
                continue  # ノイズを無視
            cluster_points = points[labels == label]
            centroid = np.mean(cluster_points, axis=0)
            clusters[label] = {'centroid': centroid, 'points': cluster_points}
        return clusters

    def update_tracking(self, previous_clusters, current_clusters, track_ids):
        previous_centroids = np.array([v['centroid'] for v in previous_clusters.values()])
        current_centroids = np.array([v['centroid'] for v in current_clusters.values()])
        cost_matrix = euclidean_distances(previous_centroids, current_centroids)
        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        new_track_ids = {}
        for r, c in zip(row_ind, col_ind):
            new_track_ids[list(current_clusters.keys())[c]] = list(track_ids.values())[r]
        
        # 新しいクラスタに新IDを付与
        new_id = max(track_ids.values()) + 1
        for label in current_clusters.keys():
            if label not in new_track_ids:
                new_track_ids[label] = new_id
                new_id += 1
        
        return new_track_ids

def main(args=None):
    rclpy.init(args=args)
    node = LaserListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
