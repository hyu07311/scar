import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('stair_mapping')
    realsense_share = get_package_share_directory('realsense2_camera')

    # 1. RealSense 카메라 실행
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_share, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'pointcloud.enable': 'true',
            'pointcloud.stream_filter': '2',      # 추가: Depth 스트림만 사용
            'pointcloud.stream_index_filter': '0',
            'enable_accel': 'false',
            'enable_gyro': 'false',
            'align_depth.enable': 'true',
            'initial_reset': 'true'
        }.items()
    )

    # 2. Stair Mapping Node
    stair_mapping_node = Node(
        package='stair_mapping',
        executable='stair_mapping_node',
        name='stair_mapping_node',
        parameters=[{
            'voxel_leaf_size': 0.02,
            'roi_max_distance': 3.0,
            'input_topic': '/camera/depth/color/points'
        }]
    )

    # 3. Map Cleaner Node
    map_cleaner_node = Node(
        package='stair_mapping',
        executable='map_cleaner_node',
        name='map_cleaner_node',
        parameters=[{'mean_k': 30, 'stddev_thresh': 1.0}]
    )

    # 4. PointCloud2 -> 2D LaserScan 변환
    pc_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{
            'target_frame': 'base_link',       # camera_link → base_link 로 수정
            'transform_tolerance': 0.01,
            'min_height': -0.5,
            'max_height': 0.5,
            'angle_min': -1.5708,
            'angle_max': 1.5708,
            'range_min': 0.3,
            'range_max': 5.0,
            'use_inf': True,
            'qos_overrides./scan.publisher.reliability': 'best_effort'
        }],
        remappings=[
            ('cloud_in', '/map_cleaned'),
            ('scan', '/scan')
        ]
    )

    # 5. TF 설정 - base_link 추가
    # odom → base_link (slam_toolbox 필수 요구)
    static_tf_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_odom_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    # base_link → camera_link
    static_tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_camera',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link']
    )

    # camera_link → camera_color_frame
    static_tf_link_to_color = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_link_to_color',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'camera_color_frame']
    )

    # camera_color_frame → camera_color_optical_frame
    static_tf_color_to_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_color_to_optical',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_color_frame', 'camera_color_optical_frame']
    )

    # 6. slam_toolbox - /map 생성 (핵심 추가)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[{
            'use_sim_time': False,
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'scan_topic': '/scan',
            'mode': 'mapping',
            'resolution': 0.05,
            'max_laser_range': 5.0,
            'minimum_travel_distance': 0.001,   # 수정: 거의 정지 상태에서도 업데이트
            'minimum_travel_heading': 0.001,    # 수정: 미세한 회전에도 업데이트
            'map_update_interval': 0.5,         # 추가: 0.5초마다 맵 업데이트
            'transform_publish_period': 0.02,
        }],
        output='screen'
    )

    # 7. Nav2 Costmap & Lifecycle Manager
    stvl_params = os.path.join(pkg_share, 'params', 'stvl_params.yaml')

    costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='costmap',
        output='screen',
        parameters=[stvl_params]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap',
        parameters=[{
            'autostart': True,
            'node_names': ['costmap']
        }]
    )

    return LaunchDescription([
        camera_launch,
        stair_mapping_node,
        map_cleaner_node,
        pc_to_scan_node,
        static_tf_odom_to_base,      # 추가
        static_tf_base_to_camera,    # 추가
        static_tf_link_to_color,
        static_tf_color_to_optical,
        slam_toolbox_node,           # 추가 (핵심)
        costmap_node,
        lifecycle_manager_node
    ])
