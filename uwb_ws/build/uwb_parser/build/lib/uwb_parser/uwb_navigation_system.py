#!/usr/bin/env python3
#일단은 안씀
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String, Int32
import math
import json


class UWBNavigationSystem(Node):
    def __init__(self):
        super().__init__('uwb_navigation_system')
        
        # 파라미터 선언
        self.declare_parameter('proximity_threshold', 50.0)  # 픽셀 단위 (50픽셀 = 10cm)
        self.declare_parameter('navigation_active', True)
        
        # 일방통행 waypoint 순서 (픽셀 좌표)
        self.waypoints = [
            (75, 250),   # waypoint 0
            (225, 250),  # waypoint 1  
            (350, 250),  # waypoint 2
            (450, 250),  # waypoint 3
            (550, 250),  # waypoint 4
            (650, 250),  # waypoint 5
            (750, 250),  # waypoint 6
            (750, 350),  # waypoint 7
            (750, 450),  # waypoint 8
            (750, 550),  # waypoint 9
            (650, 550),  # waypoint 10
            (550, 550),  # waypoint 11
            (450, 550),  # waypoint 12
            (350, 550),  # waypoint 13
            (250, 550),  # waypoint 14
        ]
        
        # 주차구역별 목적지 waypoint 매핑
        self.parking_destinations = {
            1: 0,   # 주차구역1 -> waypoint 0 (75, 250)
            2: 1,   # 주차구역2 -> waypoint 1 (225, 250)
            3: 2,   # 주차구역3 -> waypoint 2 (350, 250)
            4: 3,   # 주차구역4 -> waypoint 3 (450, 250)
            5: 4,   # 주차구역5 -> waypoint 4 (550, 250)
            6: 5,   # 주차구역6 -> waypoint 5 (650, 250)
            7: 6,   # 주차구역7 -> waypoint 6 (750, 250) - 공유
            8: 6,   # 주차구역8 -> waypoint 6 (750, 250) - 공유
            9: 7,   # 주차구역9 -> waypoint 7 (750, 350)
            10: 8,  # 주차구역10 -> waypoint 8 (750, 450)
            11: 9,  # 주차구역11 -> waypoint 9 (750, 550) - 공유
            12: 9,  # 주차구역12 -> waypoint 9 (750, 550) - 공유
            13: 10, # 주차구역13 -> waypoint 10 (650, 550)
            14: 11, # 주차구역14 -> waypoint 11 (550, 550)
            15: 12, # 주차구역15 -> waypoint 12 (450, 550)
            16: 13, # 주차구역16 -> waypoint 13 (350, 550)
            17: 14, # 주차구역17 -> waypoint 14 (250, 550)
        }
        
        # 네비게이션 상태 변수
        self.current_target_parking = None
        self.destination_waypoint_idx = None
        self.current_waypoint_idx = 0
        self.navigation_active = False
        self.proximity_threshold = self.get_parameter('proximity_threshold').value
        
        # 현재 UWB 위치
        self.current_position = None
        
        # Subscribers
        self.uwb_subscriber = self.create_subscription(
            PointStamped,
            '/uwb/coordinates',
            self.uwb_position_callback,
            10
        )
        
        self.destination_subscriber = self.create_subscription(
            Int32,
            '/navigation/set_destination',
            self.set_destination_callback,
            10
        )
        
        # Publishers
        self.navigation_status_publisher = self.create_publisher(
            String,
            '/navigation/status',
            10
        )
        
        self.current_target_publisher = self.create_publisher(
            PointStamped,
            '/navigation/current_target',
            10
        )
        
        self.path_progress_publisher = self.create_publisher(
            String,
            '/navigation/progress',
            10
        )
        
        # 타이머 (상태 업데이트용)
        self.timer = self.create_timer(0.1, self.navigation_update)
        
        self.get_logger().info('UWB Navigation System initialized')
        self.get_logger().info(f'Proximity threshold: {self.proximity_threshold} pixels')

    def set_destination_callback(self, msg):
        """목적지 주차구역 설정"""
        parking_number = msg.data
        
        if parking_number not in self.parking_destinations:
            self.get_logger().error(f'Invalid parking number: {parking_number}')
            return
        
        self.current_target_parking = parking_number
        self.destination_waypoint_idx = self.parking_destinations[parking_number]
        self.current_waypoint_idx = 0
        self.navigation_active = True
        
        # 경로 계산
        path = self.calculate_path()
        
        self.get_logger().info(f'Navigation started to Parking {parking_number}')
        self.get_logger().info(f'Path: {path}')
        
        # 네비게이션 상태 발행
        status_msg = String()
        status_msg.data = json.dumps({
            'status': 'navigation_started',
            'target_parking': parking_number,
            'destination_waypoint': self.destination_waypoint_idx,
            'path': path,
            'total_waypoints': len(path)
        })
        self.navigation_status_publisher.publish(status_msg)

    def calculate_path(self):
        """목적지까지의 경로 계산 (일방통행 순서대로)"""
        if self.destination_waypoint_idx is None:
            return []
        
        # waypoint 0부터 목적지 waypoint까지 순서대로
        path = []
        for i in range(self.destination_waypoint_idx + 1):
            path.append({
                'waypoint_idx': i,
                'coordinates': self.waypoints[i]
            })
        
        return path

    def uwb_position_callback(self, msg):
        """UWB 위치 수신 콜백"""
        self.current_position = (msg.point.x * 500, msg.point.y * 500)  # 픽셀 좌표로 변환 (0.002m -> pixel)
        
        # 디버깅용 로그
        self.get_logger().debug(f'UWB position: ({self.current_position[0]:.1f}, {self.current_position[1]:.1f})')

    def calculate_distance(self, pos1, pos2):
        """두 점 사이의 거리 계산"""
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

    def check_waypoint_reached(self):
        """현재 waypoint에 도달했는지 확인"""
        if not self.navigation_active or self.current_position is None:
            return False
        
        if self.current_waypoint_idx > self.destination_waypoint_idx:
            return False
        
        current_target = self.waypoints[self.current_waypoint_idx]
        distance = self.calculate_distance(self.current_position, current_target)
        
        return distance <= self.proximity_threshold

    def navigation_update(self):
        """네비게이션 상태 업데이트"""
        if not self.navigation_active or self.current_position is None:
            return
        
        # 현재 목표 waypoint 도달 확인
        if self.check_waypoint_reached():
            self.get_logger().info(f'Reached waypoint {self.current_waypoint_idx}: {self.waypoints[self.current_waypoint_idx]}')
            
            # 목적지 도달 확인
            if self.current_waypoint_idx == self.destination_waypoint_idx:
                self.complete_navigation()
                return
            
            # 다음 waypoint로 이동
            self.current_waypoint_idx += 1
            self.get_logger().info(f'Next target: waypoint {self.current_waypoint_idx}: {self.waypoints[self.current_waypoint_idx]}')
        
        # 현재 목표 waypoint 발행
        self.publish_current_target()
        
        # 진행 상황 발행
        self.publish_progress()

    def publish_current_target(self):
        """현재 목표 waypoint 발행"""
        if not self.navigation_active:
            return
        
        target_point = PointStamped()
        target_point.header.stamp = self.get_clock().now().to_msg()
        target_point.header.frame_id = 'map'
        
        current_target = self.waypoints[self.current_waypoint_idx]
        target_point.point.x = current_target[0] / 500.0  # 픽셀을 미터로 변환
        target_point.point.y = current_target[1] / 500.0
        target_point.point.z = 0.0
        
        self.current_target_publisher.publish(target_point)

    def publish_progress(self):
        """진행 상황 발행"""
        if not self.navigation_active or self.current_position is None:
            return
        
        progress_data = {
            'status': 'navigating',
            'current_waypoint': self.current_waypoint_idx,
            'destination_waypoint': self.destination_waypoint_idx,
            'target_parking': self.current_target_parking,
            'current_position': self.current_position,
            'target_position': self.waypoints[self.current_waypoint_idx],
            'progress_percent': (self.current_waypoint_idx / self.destination_waypoint_idx) * 100 if self.destination_waypoint_idx > 0 else 0
        }
        
        if self.current_position:
            current_target = self.waypoints[self.current_waypoint_idx]
            distance = self.calculate_distance(self.current_position, current_target)
            progress_data['distance_to_target'] = distance
        
        progress_msg = String()
        progress_msg.data = json.dumps(progress_data)
        self.path_progress_publisher.publish(progress_msg)

    def complete_navigation(self):
        """네비게이션 완료"""
        self.get_logger().info(f'Navigation completed! Arrived at Parking {self.current_target_parking}')
        
        # 완료 상태 발행
        status_msg = String()
        status_msg.data = json.dumps({
            'status': 'navigation_completed',
            'target_parking': self.current_target_parking,
            'final_position': self.current_position
        })
        self.navigation_status_publisher.publish(status_msg)
        
        # 네비게이션 상태 초기화
        self.navigation_active = False
        self.current_target_parking = None
        self.destination_waypoint_idx = None
        self.current_waypoint_idx = 0

    def cancel_navigation(self):
        """네비게이션 취소"""
        if self.navigation_active:
            self.get_logger().info('Navigation cancelled')
            
            status_msg = String()
            status_msg.data = json.dumps({
                'status': 'navigation_cancelled'
            })
            self.navigation_status_publisher.publish(status_msg)
        
        self.navigation_active = False
        self.current_target_parking = None
        self.destination_waypoint_idx = None
        self.current_waypoint_idx = 0


def main(args=None):
    rclpy.init(args=args)
    
    navigation_system = UWBNavigationSystem()
    
    try:
        rclpy.spin(navigation_system)
    except KeyboardInterrupt:
        navigation_system.get_logger().info('Navigation system shutting down...')
    finally:
        navigation_system.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()