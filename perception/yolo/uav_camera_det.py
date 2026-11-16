"""
Onboard AI detection for the drone camera.

Subscribes to the camera topic, runs YOLOv8, then:
  - sends each detection to the ground station as JSON over UDP (live),
  - appends it to data/inspection_report.csv with the drone's position (report).

Environment variables (all optional):
  YOLO_WEIGHTS          weights file, default yolov8m.pt (use custom weights for damage)
  YOLO_CLASSES          comma-separated class ids to keep, default "0,2" (person, car)
  YOLO_DEVICE           force "cpu", "mps" (Apple Silicon GPU) or "cuda"; default auto
  YOLO_VIEW             "1" shows the OpenCV window; set "0" on headless machines
  GROUND_STATION_HOST   default 127.0.0.1 (use host.docker.internal from Docker on macOS)
  GROUND_STATION_PORT   default 14560
  REPORT_CSV            default data/inspection_report.csv at the repo root
"""

import csv
import math
import os
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from ultralytics import YOLO

from px4_orbit_inspection.ground_link import GroundLink, classify, make_event

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
REPORT_HEADER = ['image_time_s', 'class', 'confidence', 'north_m', 'east_m', 'altitude_m',
                 'inference_ms', 'bbox_xyxy']


def pick_device():
    """Use the Apple GPU (MPS) on an M-series Mac, CUDA on NVIDIA, otherwise the CPU."""
    forced = os.environ.get('YOLO_DEVICE')
    if forced:
        return forced
    import torch
    if torch.cuda.is_available():
        return 'cuda:0'
    if getattr(torch.backends, 'mps', None) and torch.backends.mps.is_available():
        return 'mps'
    return 'cpu'


def _fmt(value):
    """Round a float for the report; empty cell when the value is NaN (no odometry yet)."""
    return '' if math.isnan(value) else round(value, 2)


class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')

        weights = os.environ.get('YOLO_WEIGHTS', 'yolov8m.pt')
        if not os.path.isabs(weights):
            weights = os.path.join(HERE, weights)
        self.device = pick_device()
        self.model = YOLO(weights)
        self.classes = [int(c) for c in os.environ.get('YOLO_CLASSES', '0,2').split(',') if c.strip()]
        self.show = os.environ.get('YOLO_VIEW', '1') == '1'
        self.get_logger().info(f'YOLO on device "{self.device}", weights {os.path.basename(weights)}')

        self.link = GroundLink(os.environ.get('GROUND_STATION_HOST', '127.0.0.1'),
                               int(os.environ.get('GROUND_STATION_PORT', '14560')))
        self.report_path = os.environ.get('REPORT_CSV', os.path.join(REPO_ROOT, 'data', 'inspection_report.csv'))
        self._ensure_report_header()

        self.position = (float('nan'), float('nan'), float('nan'))  # NED from PX4 odometry

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self._odom, sensor_qos)
        self.create_subscription(Image, 'camera', self.listener_callback, 10)
        self.br = CvBridge()

    def _odom(self, msg):
        self.position = tuple(float(v) for v in msg.position)

    def _ensure_report_header(self):
        if not os.path.exists(self.report_path) or os.path.getsize(self.report_path) == 0:
            os.makedirs(os.path.dirname(self.report_path), exist_ok=True)
            with open(self.report_path, 'w', newline='') as f:
                csv.writer(f).writerow(REPORT_HEADER)

    def listener_callback(self, data):
        image = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        started = time.perf_counter()
        results = self.model.predict(image, classes=self.classes, device=self.device, verbose=False)
        inference_ms = round((time.perf_counter() - started) * 1000, 1)
        result = results[0]
        north, east, down = self.position
        altitude = -down

        rows, events = [], []
        for box, conf, cls in zip(result.boxes.xyxy.tolist(), result.boxes.conf.tolist(),
                                  result.boxes.cls.tolist()):
            label = result.names[int(cls)]
            rows.append([round(time.time(), 3), label, round(conf, 3),
                         _fmt(north), _fmt(east), _fmt(altitude), inference_ms,
                         ' '.join(str(round(v)) for v in box)])
            events.append(make_event('detection', category=classify(label), label=label,
                                     confidence=round(conf, 3), bbox_xyxy=[round(v) for v in box],
                                     north_m=_fmt(north), east_m=_fmt(east), altitude_m=_fmt(altitude)))

        if rows:
            with open(self.report_path, 'a', newline='') as f:
                csv.writer(f).writerows(rows)
            for event in events:
                self.link.send(event)
            self.get_logger().info(f'{len(rows)} detection(s) in {inference_ms} ms')

        if self.show:
            cv2.imshow('Detected Frame', result.plot())
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
