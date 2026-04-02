"""
scar_manual.launch.py
──────────────────────────────────────────────────────────
SCAR 수동 조작 전용 런치

scar_full.launch.py 대비 제외 항목:
  - scar_navigation  (Nav2 planner/controller)
  - scar_vision      (person_node, stair_node)
  - my_stair_mapping (SLAM — RealSense도 생략)

포함 항목:
  - micro_ros_agent    (OpenCR 연결)
  - scar_state_manager (keyboard 모드로 직접 조작)

용도:
  - 하드웨어 초기 연결 확인
  - 실측 보정 (로드셀 scale, 액추에이터 속도, 슬라이드 delta 등)
  - AUTO 모드 진입 전 수동 기능 점검
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    scar_params = os.path.join(
        get_package_share_directory('scar_bringup'),
        'config', 'scar_params.yaml'
    )

    # ── micro_ros_agent (OpenCR USB) ──────────────────────
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/ttyACM0', '-b', '115200'],
        output='screen'
    )

    # ── scar_state_manager ────────────────────────────────
    # keyboard 모드로 W/S/A/D/Q/E/I/K/B/F/[/] 직접 제어
    scar_state_manager = Node(
        package='scar_state_manager',
        executable='scar_state_manager',
        name='scar_state_manager',
        output='screen',
        parameters=[scar_params]
    )

    return LaunchDescription([
        micro_ros_agent,
        scar_state_manager,
    ])
