"""
scar_full.launch.py
──────────────────────────────────────────────────────────
SCAR 전체 시스템 통합 런치

실행 순서:
  1. micro_ros_agent          — OpenCR USB 연결 (scar_cmd ↔ scar_status)
  2. my_stair_mapping          — RealSense + PointCloud 처리 + SLAM (/map, /scan)
  3. scar_navigation           — Nav2 full stack (planner, controller, BT)
  4. scar_vision person_node   — 인체 감지 → /human_detected
  5. scar_vision stair_node    — 계단 정렬 → /stair_lateral_error
  6. scar_state_manager        — 상태 머신 + IK (scar_params.yaml 로드)

주의:
  - OpenCR 연결 포트는 환경에 따라 /dev/ttyACM0 또는 /dev/ttyUSB0 확인 필요
  - RealSense는 my_stair_mapping 런치 내부에서 실행됨
    (RGB + depth + point cloud 모두 활성화)
  - scar_vision은 /camera/color/image_raw, /camera/depth/image_rect_raw 필요
    → my_stair_mapping의 realsense 런치가 이미 align_depth.enable:true 이므로 제공됨
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    scar_bringup_share  = get_package_share_directory('scar_bringup')
    my_stair_share      = get_package_share_directory('my_stair_mapping')
    scar_nav_share      = get_package_share_directory('scar_navigation')

    scar_params         = os.path.join(scar_bringup_share,  'config', 'scar_params.yaml')
    stair_nav_launch    = os.path.join(my_stair_share,      'launch', 'stair_nav.launch.py')
    nav2_launch         = os.path.join(scar_nav_share,      'launch', 'navigation.launch.py')

    # ── 1. micro_ros_agent (OpenCR USB) ───────────────────
    # serial 모드: OpenCR이 /dev/ttyACM0 에 연결된 경우
    # 다른 포트라면 args의 -p 값을 변경
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/ttyACM0', '-b', '115200'],
        output='screen'
    )

    # ── 2. my_stair_mapping (RealSense + SLAM) ─────────────
    # 내부에서 realsense2_camera, stair_mapping_node, map_cleaner_node,
    # pointcloud_to_laserscan, slam_toolbox, static TF 를 실행함
    stair_mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(stair_nav_launch)
    )

    # ── 3. scar_navigation (Nav2 full stack) ───────────────
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch)
    )

    # ── 4. person_node (인체 감지 → /human_detected) ───────
    person_node = Node(
        package='scar_vision',
        executable='person_node',
        name='person_node',
        output='screen'
    )

    # ── 5. stair_node (계단 정렬 오차 → /stair_lateral_error) ─
    stair_node = Node(
        package='scar_vision',
        executable='stair_node',
        name='stair_node',
        output='screen'
    )

    # ── 6. scar_state_manager (상태 머신 + IK) ─────────────
    # scar_params.yaml에서 wheel_radius=0.12, wheel_base_x=0.72,
    # wheel_track=0.34 등 실측값을 로드함
    scar_state_manager = Node(
        package='scar_state_manager',
        executable='scar_state_manager',
        name='scar_state_manager',
        output='screen',
        parameters=[scar_params]
    )

    return LaunchDescription([
        micro_ros_agent,
        stair_mapping_launch,
        navigation_launch,
        person_node,
        stair_node,
        scar_state_manager,
    ])
