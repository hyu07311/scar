#!/usr/bin/env python3
"""
stair_node.py
  계단 좌측 정렬 오차 계산 전용 노드

  클래스: 0=Downstairs, 1=Upstairs, 2=person (best.pt 기준)

  발행 토픽:
    /stair_lateral_error (std_msgs/Float32, 단위: m)
      양수 = 계단이 로봇 우측에 있음 → 로봇이 좌측으로 이동해야
      음수 = 계단이 로봇 좌측에 있음 → 로봇이 우측으로 이동해야
      0    = 계단 좌측 끝이 카메라 중앙과 정렬됨

  계산 원리:
    계단 bbox 좌측 끝(x1) 기준으로 카메라 중앙과의 픽셀 오차를
    depth 값과 카메라 intrinsic(fx)으로 실제 거리(m)로 변환
    lateral_error = (pixel_error × depth) / fx
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np

# 계단 클래스 ID (best.pt 기준)
CLS_DOWNSTAIRS = 0
CLS_UPSTAIRS   = 1


class StairNode(Node):
    def __init__(self):
        super().__init__('stair_node')

        # 파라미터
        self.declare_parameter('model_path',    '/home/scar/ros2_ws/src/scar_vision/weights/best.engine')
        self.declare_parameter('conf_threshold', 0.60)
        self.declare_parameter('depth_min_m',    0.3)  # 유효 depth 최솟값
        self.declare_parameter('depth_max_m',    5.0)  # 유효 depth 최댓값

        model_path       = self.get_parameter('model_path').value
        self.conf_thr    = self.get_parameter('conf_threshold').value
        self.depth_min   = self.get_parameter('depth_min_m').value
        self.depth_max   = self.get_parameter('depth_max_m').value

        # YOLOv8 로드
        self.model  = YOLO(model_path)
        self.bridge = CvBridge()

        # 카메라 intrinsic
        # CameraInfo 수신 전까지 발행 보류
        self.fx = None
        self.cx = None

        # depth 이미지 버퍼
        self.latest_depth = None

        # 구독
        self.create_subscription(
            Image, '/camera/color/image_raw',
            self.rgb_cb, 10)
        self.create_subscription(
            Image, '/camera/depth/image_rect_raw',
            self.depth_cb, 10)
        self.create_subscription(
            CameraInfo, '/camera/color/camera_info',
            self.info_cb, 10)

        # 발행
        self.stair_pub = self.create_publisher(
            Float32, '/stair_lateral_error', 10)

        self.get_logger().info(
            f'StairNode 시작 — conf={self.conf_thr} '
            f'depth=[{self.depth_min},{self.depth_max}]m')

    # ── 카메라 intrinsic 수신 (1회) ───────────────────────────
    def info_cb(self, msg):
        if self.fx is None:
            self.fx = msg.k[0]  # 초점거리 (픽셀)
            self.cx = msg.k[2]  # 주점 x   (픽셀)
            self.get_logger().info(
                f'카메라 intrinsic 수신: fx={self.fx:.1f} cx={self.cx:.1f}')

    # ── depth 이미지 버퍼 ────────────────────────────────────
    def depth_cb(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough')

    # ── RGB 수신 → 추론 → 오차 발행 ─────────────────────────
    def rgb_cb(self, msg):
        # intrinsic 또는 depth 미수신 시 스킵
        if self.fx is None or self.latest_depth is None:
            return

        img     = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(img, conf=self.conf_thr, verbose=False)

        # 신뢰도 가장 높은 계단 bbox 선택
        best_box  = None
        best_conf = 0.0

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                conf   = float(box.conf[0])

                # Downstairs(0) / Upstairs(1) 모두 계단으로 처리
                if cls_id not in (CLS_DOWNSTAIRS, CLS_UPSTAIRS):
                    continue

                if conf > best_conf:
                    best_conf = conf
                    best_box  = box

        if best_box is None:
            # 계단 미감지 → 발행 안 함
            # scar_state_manager VISION_ALIGN에서 타임아웃으로 처리
            return

        x1, y1, x2, y2 = map(int, best_box.xyxy[0])

        # 계단 좌측 끝(x1)을 정렬 기준으로 사용
        # SCAR는 계단 좌측 끝에서 우측 방향으로 청소하므로
        stair_left_x = x1
        y_center     = (y1 + y2) // 2

        # depth 유효성 체크
        h, w = self.latest_depth.shape[:2]
        if not (0 <= y_center < h and 0 <= stair_left_x < w):
            return

        depth_m = self.latest_depth[y_center, stair_left_x] / 1000.0

        if not (self.depth_min < depth_m < self.depth_max):
            self.get_logger().warn(
                f'[STAIR] depth 유효 범위 벗어남: {depth_m:.2f}m',
                throttle_duration_sec=1.0)
            return

        # 픽셀 오차 → 실제 횡방향 거리 (m)
        # 삼각형 닮음: lateral_error = pixel_error × depth / fx
        pixel_error   = float(stair_left_x) - self.cx
        lateral_error = (pixel_error * depth_m) / self.fx

        self.stair_pub.publish(Float32(data=lateral_error))

        self.get_logger().debug(
            f'[STAIR] x1={x1} depth={depth_m:.2f}m '
            f'pixel_err={pixel_error:.1f} '
            f'lateral_err={lateral_error:.3f}m')


def main(args=None):
    rclpy.init(args=args)
    node = StairNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
