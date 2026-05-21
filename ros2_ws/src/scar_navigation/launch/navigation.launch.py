"""
navigation.launch.py
──────────────────────────────────────────────────────────
Nav2 full stack 실행 (SCAR 전용)

전제:
  - /scan (LaserScan)  ─┐ my_stair_mapping 또는
  - /map  (OccupancyGrid)┘ scar_full.launch.py 가 먼저 제공해야 함
  - map → odom → base_link  TF 체인이 이미 존재해야 함

포함 노드:
  controller_server  (DWB 로컬 컨트롤러)
  planner_server     (NavFn 글로벌 플래너)
  recoveries_server  (spin / backup / wait)
  bt_navigator
  lifecycle_manager  (위 4개 노드 관리)

발행 토픽:
  /cmd_vel → scar_state_manager IDLE 상태에서 IK 통과
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    nav2_params = os.path.join(
        get_package_share_directory('scar_navigation'),
        'config', 'nav2_params.yaml'
    )

    # ── controller_server (DWB 로컬 플래너 + 로컬 코스트맵) ──
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params]
    )

    # ── planner_server (NavFn 글로벌 플래너 + 글로벌 코스트맵) ─
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params]
    )

    # ── recoveries_server (spin / backup / wait) ───────────
    recoveries_server = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[nav2_params]
    )

    # ── bt_navigator ───────────────────────────────────────
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params]
    )

    # ── lifecycle_manager: 위 4개 노드를 순서대로 활성화 ───
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'recoveries_server',
                'bt_navigator',
            ]
        }]
    )

    return LaunchDescription([
        controller_server,
        planner_server,
        recoveries_server,
        bt_navigator,
        lifecycle_manager,
    ])
