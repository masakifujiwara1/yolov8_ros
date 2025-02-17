# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import numpy as np
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

import message_filters
from cv_bridge import CvBridge

from ultralytics.trackers import BOTSORT, BYTETracker
from ultralytics.trackers.basetrack import BaseTrack
from ultralytics.utils import IterableSimpleNamespace, yaml_load
from ultralytics.utils.checks import check_requirements, check_yaml
from ultralytics.engine.results import Boxes

from sensor_msgs.msg import Image, CompressedImage
from yolov8_msgs.msg import Detection
from yolov8_msgs.msg import DetectionArray


class TrackingNode(Node):

    def __init__(self) -> None:
        super().__init__("tracking_node")

        # params
        self.declare_parameter("tracker", "bytetrack.yaml")
        self.declare_parameter("image_reliability",
                               QoSReliabilityPolicy.BEST_EFFORT)

        self.cv_bridge = CvBridge()

        tracker_name = self.get_parameter(
            "tracker").get_parameter_value().string_value

        self.image_reliability = self.get_parameter(
            "image_reliability").get_parameter_value().integer_value

        self.tracker = self.create_tracker(tracker_name)
        self._pub = self.create_publisher(DetectionArray, "tracking", 10)

        image_qos_profile = QoSProfile(
            reliability=self.image_reliability,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # subs
        self.image_sub = message_filters.Subscriber(
            self, CompressedImage, "image_raw", qos_profile=image_qos_profile)
        self.detections_sub = message_filters.Subscriber(
            self, DetectionArray, "detections", qos_profile=10)

        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.detections_sub], 10, 0.5)
        self._synchronizer.registerCallback(self.detections_cb)

    def create_tracker(self, tracker_yaml: str) -> BaseTrack:

        TRACKER_MAP = {"bytetrack": BYTETracker, "botsort": BOTSORT}
        check_requirements("lap")  # for linear_assignment

        tracker = check_yaml(tracker_yaml)
        cfg = IterableSimpleNamespace(**yaml_load(tracker))

        assert cfg.tracker_type in ["bytetrack", "botsort"], \
            f"Only support 'bytetrack' and 'botsort' for now, but got '{cfg.tracker_type}'"
        tracker = TRACKER_MAP[cfg.tracker_type](args=cfg, frame_rate=1)
        return tracker

    def calc_theta(self, w_img_, x_center_):
        fov_horizonal = math.pi / 2 # 90deg
        # theta_ = -((x_center_ - (w_img_/2)) * 3.1415) / (w_img_/2)
        theta_ = -((x_center_ - (w_img_ / 2)) * fov_horizonal) / w_img_
        theta_ = math.atan2(math.sin(theta_), math.cos(theta_))

        # half_w_img_ = (w_img_ / 2)
        # l = half_w_img_ / math.tan(fov_horizonal / 2)
        # if x_center_ > half_w_img_:
        # theta_ = math.atan((x_center_ - half_w_img_) / l)
        # else:
        #     theta_ = -math.atan((x_center_ - half_w_img_) / l)
        # theta_ = math.atan2(math.sin(theta_), math.cos(theta_))

        # self.get_logger().info(f'x_center_: {x_center_}, theta: {theta_}')

        return theta_

    def detections_cb(self, img_msg: CompressedImage, detections_msg: DetectionArray) -> None:

        tracked_detections_msg = DetectionArray()
        tracked_detections_msg.header = img_msg.header

        # convert image
        cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(img_msg)

        # parse detections
        detection_list = []
        detection: Detection
        for detection in detections_msg.detections:

            detection_list.append(
                [
                    detection.bbox.center.position.x - detection.bbox.size.x / 2,
                    detection.bbox.center.position.y - detection.bbox.size.y / 2,
                    detection.bbox.center.position.x + detection.bbox.size.x / 2,
                    detection.bbox.center.position.y + detection.bbox.size.y / 2,
                    detection.score,
                    detection.class_id
                ]
            )

        img_msg = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")

        # tracking
        if len(detection_list) > 0:

            det = Boxes(
                np.array(detection_list),
                (img_msg.height, img_msg.width) # 720, 1280 fov H: 90 V: 59
            )

            # self.get_logger().info(str(img_msg.height) + str(img_msg.width))

            tracks = self.tracker.update(det, cv_image)

            if len(tracks) > 0:

                for t in tracks:

                    tracked_box = Boxes(
                        t[:-1], (img_msg.height, img_msg.width))

                    tracked_detection: Detection = detections_msg.detections[int(
                        t[-1])]

                    # get boxes values
                    box = tracked_box.xywh[0]
                    tracked_detection.bbox.center.position.x = float(box[0])
                    tracked_detection.bbox.center.position.y = float(box[1])

                    tracked_detection.bbox.center.theta = self.calc_theta(img_msg.width, float(box[0]))

                    tracked_detection.bbox.size.x = float(box[2])
                    tracked_detection.bbox.size.y = float(box[3])

                    # get track id
                    track_id = ""
                    if tracked_box.is_track:
                        track_id = str(int(tracked_box.id))
                    tracked_detection.id = track_id

                    # append msg
                    tracked_detections_msg.detections.append(tracked_detection)

        # publish detections
        self._pub.publish(tracked_detections_msg)


def main():
    rclpy.init()
    node = TrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
