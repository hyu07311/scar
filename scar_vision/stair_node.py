#!/usr/bin/env python3
"""
stair_node.py
  계단 좌측 정렬 오차 + map 좌표 계산 노드

  클래스: 0=Downstairs, 1=Upstairs, 2=person (best.pt 기준)

  발행 토픽:
    /stair_lateral_error (std_msgs/Float32, 단위: m)
      양수 = 계단이 로봇 우측에 있음 → 로봇이 좌측으로 이동해야
      음수 = 계단이 로봇 좌측에 있음 → 로봇이 우측으로 이동해야
      0    = 계단 좌측 끝이 카메라 중앙과 정렬됨

    /stair_goal_pose (geometry_msgs/PoseStamped, frame_id="map")
      계단 bbox 중심의 map 좌표 — Nav2 목표 지점으로 사용

  계산 원리:
    [횡방향 오차]
      계단 bbox 좌측 끝(x1) 기준으로 카메라 중앙과의 픽셀 오차를
      depth 값과 카메라 intrinsic(fx)으로 실제 거리(m)로 변환
      lateral_error = (pixel_error × depth) / fx

    [map 좌표]
      bbox 중심 픽셀의 depth로 3D 좌표 복원 (camera_color_optical_frame)
        X = (u - cx) * depth / fx
        Y = (v - cy) * depth / fy
        Z = depth
      TF2로 map frame 변환 → PoseStamped 발행
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — register PoseStamped transform

# 계단 클래스 ID (best.pt 기준)
CLS_DOWNSTAIRS = 0
CLS_UPSTAIRS   = 1


class StairNode(Node):
    def __init__(self):
        super().__init__('stair_node')

        # 파라미터
        self.declare_parameter('model_path',    '/home/scar/ros2_ws/src/scar_vision/weights/best.engine')
        self.declare_parameter('conf_threshold', 0.60)
        self.declare_parameter('depth_min_m',    0.3)
        self.declare_parameter('depth_max_m',    5.0)

        model_path       = self.get_parameter('model_path').value
        self.conf_thr    = self.get_parameter('conf_threshold').value
        self.depth_min   = self.get_parameter('depth_min_m').value
        self.depth_max   = self.get_parameter('depth_max_m').value

        # YOLOv8 로드
        self.model  = YOLO(model_path)
        self.bridge = CvBridge()

        # 카메라 intrinsic (CameraInfo 수신 전까지 발행 보류)
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # depth 이미지 버퍼
        self.latest_depth = None

        # TF2 — camera_color_optical_frame → map 변환
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

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
        self.goal_pub  = self.create_publisher(
            PoseStamped, '/stair_goal_pose', 10)

        self.get_logger().info(
            f'StairNode 시작 — conf={self.conf_thr} '
            f'depth=[{self.depth_min},{self.depth_max}]m')

    # ── 카메라 intrinsic 수신 (1회) ───────────────────────────
    def info_cb(self, msg):
        if self.fx is None:
            self.fx = msg.k[0]   # 초점거리 x (픽셀)
            self.fy = msg.k[4]   # 초점거리 y (픽셀)
            self.cx = msg.k[2]   # 주점 x     (픽셀)
            self.cy = msg.k[5]   # 주점 y     (픽셀)
            self.camera_frame = msg.header.frame_id  # camera_color_optical_frame
            self.get_logger().info(
                f'카메라 intrinsic 수신: fx={self.fx:.1f} fy={self.fy:.1f} '
                f'cx={self.cx:.1f} cy={self.cy:.1f} '
                f'frame={self.camera_frame}')

    # ── depth 이미지 버퍼 ────────────────────────────────────
    def depth_cb(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough')

    # ── RGB 수신 → 추론 → 오차/좌표 발행 ────────────────────
    def rgb_cb(self, msg):
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
                if cls_id not in (CLS_DOWNSTAIRS, CLS_UPSTAIRS):
                    continue
                if conf > best_conf:
                    best_conf = conf
                    best_box  = box

        if best_box is None:
            return

        x1, y1, x2, y2 = map(int, best_box.xyxy[0])

        # ── [1] 횡방향 오차 발행 (/stair_lateral_error) ──────
        # 계단 좌측 끝(x1)을 정렬 기준으로 사용
        stair_left_x = x1
        y_center_l   = (y1 + y2) // 2

        h, w = self.latest_depth.shape[:2]
        if 0 <= y_center_l < h and 0 <= stair_left_x < w:
            depth_l = self.latest_depth[y_center_l, stair_left_x] / 1000.0
            if self.depth_min < depth_l < self.depth_max:
                pixel_error   = float(stair_left_x) - self.cx
                lateral_error = (pixel_error * depth_l) / self.fx
                self.stair_pub.publish(Float32(data=lateral_error))
                self.get_logger().debug(
                    f'[STAIR] lateral_err={lateral_error:.3f}m '
                    f'depth={depth_l:.2f}m')
            else:
                self.get_logger().warn(
                    f'[STAIR] lateral depth 범위 벗어남: {depth_l:.2f}m',
                    throttle_duration_sec=1.0)

        # ── [2] map 좌표 발행 (/stair_goal_pose) ─────────────
        # bbox 중심 픽셀로 3D 좌표 복원
        u_c = (x1 + x2) // 2
        v_c = (y1 + y2) // 2

        if not (0 <= v_c < h and 0 <= u_c < w):
            return

        depth_c = self.latest_depth[v_c, u_c] / 1000.0
        if not (self.depth_min < depth_c < self.depth_max):
            self.get_logger().warn(
                f'[STAIR] goal depth 범위 벗어남: {depth_c:.2f}m',
                throttle_duration_sec=1.0)
            return

        # 핀홀 역투영: camera_color_optical_frame 기준 3D 좌표
        X_cam = (u_c - self.cx) * depth_c / self.fx
        Y_cam = (v_c - self.cy) * depth_c / self.fy
        Z_cam = depth_c

        # PointStamped로 TF2 변환
        point_cam = PointStamped()
        point_cam.header.stamp    = msg.header.stamp
        point_cam.header.frame_id = self.camera_frame
        point_cam.point.x = X_cam
        point_cam.point.y = Y_cam
        point_cam.point.z = Z_cam

        try:
            point_map = self.tf_buffer.transform(
                point_cam, 'map', timeout=rclpy.duration.Duration(seconds=0.1))
        except Exception as e:
            self.get_logger().warn(
                f'[STAIR] TF 변환 실패: {e}',
                throttle_duration_sec=1.0)
            return

        # PoseStamped 발행 (방향은 정면 기본값 사용)
        goal = PoseStamped()
        goal.header.stamp    = point_map.header.stamp
        goal.header.frame_id = 'map'
        goal.pose.position.x = point_map.point.x
        goal.pose.position.y = point_map.point.y
        goal.pose.position.z = 0.0          # 2D 네비게이션 — z=0 고정
        goal.pose.orientation.w = 1.0       # 방향: 정면 (yaw=0)
        self.goal_pub.publish(goal)

        self.get_logger().debug(
            f'[STAIR] goal map=({goal.pose.position.x:.2f}, '
            f'{goal.pose.position.y:.2f})')


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
