#!/usr/bin/env python3
"""
person_node.py
  인체 감지 전용 노드

  클래스: 0=Downstairs, 1=Upstairs, 2=person (best.pt 기준)

  발행 토픽:
    /human_detected  (std_msgs/Bool)    → scar_state_manager 연동
    /person_dist     (std_msgs/Float32) → 디버깅용 거리값
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import time

CLS_PERSON = 2  # best.pt 기준: {0:Downstairs, 1:Upstairs, 2:person}


class PersonNode(Node):
    def __init__(self):
        super().__init__('person_node')

        # 파라미터
        self.declare_parameter('model_path',     '/home/scar/ros2_ws/src/scar_vision/weights/best.engine')
        self.declare_parameter('conf_threshold',  0.65)
        self.declare_parameter('stop_distance',   3.0)   # m, 이 거리 이내 감지 시 정지
        self.declare_parameter('resume_wait_sec', 7.0)   # 정지 후 재개 대기 시간

        model_path       = self.get_parameter('model_path').value
        self.conf_thr    = self.get_parameter('conf_threshold').value
        self.stop_dist   = self.get_parameter('stop_distance').value
        self.resume_wait = self.get_parameter('resume_wait_sec').value

        # YOLOv8 로드
        self.model  = YOLO(model_path)
        self.bridge = CvBridge()

        # depth 이미지 버퍼
        self.latest_depth = None

        # 인체 감지 상태 (ensure_safety 로직)
        self.is_stopped    = False
        self.stop_time     = 0.0
        self.last_dist     = -1.0
        self.last_update_t = 0.0

        # 구독
        self.create_subscription(
            Image, '/camera/color/image_raw',
            self.rgb_cb, 10)
        self.create_subscription(
            Image, '/camera/depth/image_rect_raw',
            self.depth_cb, 10)

        # 발행
        self.human_pub = self.create_publisher(Bool,    '/human_detected', 10)
        self.dist_pub  = self.create_publisher(Float32, '/person_dist',    10)

        # 0.1초 상태 체크 타이머 (ensure_safety 로직)
        self.create_timer(0.1, self.check_state)

        self.get_logger().info(
            f'PersonNode 시작 — conf={self.conf_thr} '
            f'stop_dist={self.stop_dist}m resume={self.resume_wait}s')

    def depth_cb(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough')

    def rgb_cb(self, msg):
        if self.latest_depth is None:
            return

        img     = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(img, conf=self.conf_thr, verbose=False)

        person_detected = False
        min_dist        = float('inf')

        for r in results:
            for box in r.boxes:
                if int(box.cls[0]) != CLS_PERSON:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2

                h, w = self.latest_depth.shape[:2]
                if not (0 <= cy < h and 0 <= cx < w):
                    continue

                dist_m = self.latest_depth[cy, cx] / 1000.0

                # 유효 거리 범위 (0.3m ~ 8m)
                if not (0.3 < dist_m < 8.0):
                    continue

                person_detected = True
                min_dist = min(min_dist, dist_m)

        if person_detected:
            self.last_dist     = min_dist
            self.last_update_t = time.time()

            # 이동 중 stop_dist 이내 감지 → 즉시 정지
            if not self.is_stopped and self.last_dist <= self.stop_dist:
                self.get_logger().warn(
                    f'[PERSON] 인체 감지 {self.last_dist:.2f}m → 정지')
                self.is_stopped = True
                self.stop_time  = time.time()

            self.dist_pub.publish(Float32(data=float(self.last_dist)))

        # /human_detected 매 프레임 발행
        self.human_pub.publish(Bool(data=self.is_stopped))

    def check_state(self):
        # 0.5초 이상 데이터 없으면 사람이 사라진 것으로 간주
        if time.time() - self.last_update_t > 0.5:
            self.last_dist = -1.0

        if not self.is_stopped:
            return

        # resume_wait 초 경과 후 안전 여부 판단
        elapsed = time.time() - self.stop_time
        if elapsed >= self.resume_wait:
            if self.last_dist > self.stop_dist or self.last_dist < 0:
                self.get_logger().info('[PERSON] 안전 확보 → 재개')
                self.is_stopped = False
            else:
                self.get_logger().info(
                    f'[PERSON] 대기 중 (거리: {self.last_dist:.2f}m)',
                    throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = PersonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
