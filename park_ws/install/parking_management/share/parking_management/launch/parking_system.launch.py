# park_ws/src/parking_management/launch/parking_system.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # 런치 아규먼트 선언
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('parking_management'),
                'config',
                'parking_config.yaml'
            ]),
            description='주차장 설정 파일 경로'
        ),
        
        DeclareLaunchArgument(
            'teammate_ip',
            default_value='192.168.1.100',
            description='팀원 노트북 IP 주소'
        ),
        
        DeclareLaunchArgument(
            'teammate_port',
            default_value='9999',
            description='팀원 노트북 포트 번호'
        ),
        
        DeclareLaunchArgument(
            'wifi_port',
            default_value='8888',
            description='WiFi 서버 포트 번호'
        ),
        
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='로그 레벨 (debug, info, warn, error)'
        ),
        
        # 시작 메시지
        LogInfo(
            msg=['🚗 스마트 주차장 관제 시스템을 시작합니다...']
        ),
        
        LogInfo(
            msg=[
                '📍 팀원 노트북: ', 
                LaunchConfiguration('teammate_ip'), 
                ':', 
                LaunchConfiguration('teammate_port')
            ]
        ),
        
        # 주차장 관제 메인 노드
        Node(
            package='parking_management',
            executable='parking_management_node',
            name='parking_management_node',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {
                    'teammate_ip': LaunchConfiguration('teammate_ip'),
                    'teammate_port': LaunchConfiguration('teammate_port'),
                    'wifi_port': LaunchConfiguration('wifi_port'),
                }
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            remappings=[
                # UWB 위치 데이터 (다른 패키지에서 받아올 경우)
                ('/uwb/position', '/positioning/current_position'),
                # 네비게이션 시스템으로 waypoint 전송
                ('/parking/waypoints', '/navigation/planned_path'),
                # 시스템 상태
                ('/parking/status', '/system/parking_status'),
            ]
        ),
        
        # 주차장 모니터링 노드 (선택사항)
        Node(
            package='parking_management',
            executable='parking_monitor_node',
            name='parking_monitor_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
            condition='if'  # 필요시 활성화: launch 시 use_monitor:=true 추가
        ),
        
        # 완료 메시지
        LogInfo(
            msg=['✅ 주차장 관제 시스템이 시작되었습니다!']
        ),
        
        LogInfo(
            msg=['🔗 WiFi 서버가 포트 ', LaunchConfiguration('wifi_port'), '에서 대기 중...']
        ),
        
        LogInfo(
            msg=['📱 팀원의 노트북에서 클라이언트를 실행하세요.']
        ),
    ])