#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
import json
import time
import threading
from datetime import datetime
import math

class NormalVehicleTest4Dummy(Node):
    def __init__(self):
        super().__init__('normal_vehicle_test4_dummy')
        
        # UWB 좌표 발행 (주차장 모니터링 프로그램이 /uwb/comp 구독)
        self.uwb_comp_pub = self.create_publisher(
            PointStamped, '/uwb/comp', 10)
        
        # 차량 정보 발행 (일반차량으로 설정)
        self.vehicle_info_pub = self.create_publisher(
            String, '/uwb/vehicle_info', 10)
        
        # 테스트 변수
        self.tag_id = 88  # 테스트용 태그 ID
        self.vehicle_id = "TEST일반차4번"
        self.current_position = [200.0, 200.0]  # 시작점 (입구)
        self.movement_speed = 25.0  # 픽셀/초 (빠른 이동)
        self.is_running = False
        
        # 4번 주차장까지의 경로
        self.route_points = [
            (200, 200),    # 시작점 (입구)
            (200, 925),    # 필수 경유점
            (200, 1475),   # 상단으로 직진
            (900, 1475),   # 4번 주차구역 웨이포인트
            (900, 1800),   # 4번 주차구역 중앙 감지구역 (목표)
        ]
        
        self.current_target_index = 0
        
        self.get_logger().info('일반차량 4번 주차구역 테스트 더미 노드 시작')
        self.get_logger().info(f'테스트 시나리오: 일반차량({self.vehicle_id})이 4번 주차구역에 주차')
        self.get_logger().info('테스트 시작을 위해 start_test() 메서드를 호출하세요')

    def start_test(self):
        """테스트 시작"""
        if self.is_running:
            self.get_logger().warn('테스트가 이미 실행 중입니다')
            return
            
        self.is_running = True
        self.get_logger().info('=== 일반차량 4번 주차구역 테스트 시작 ===')
        
        # 1. 차량 추적 시작 정보 발행 (일반차량으로)
        self.publish_vehicle_start_info()
        
        # 2. 경로를 따라 이동
        self.move_along_route()

    def stop_test(self):
        """테스트 중지"""
        self.is_running = False
        self.get_logger().info('테스트 중지')

    def publish_vehicle_start_info(self):
        """차량 추적 시작 정보 발행 (일반차량으로 설정)"""
        vehicle_info = {
            "tag_id": self.tag_id,
            "vehicle_id": self.vehicle_id,
            "elec": False,      # 전기차 아님
            "disabled": False,  # 장애인 차량 아님 (일반차량)
            "owner": "테스트사용자",
            "action": "start_tracking"
        }
        
        vehicle_info_msg = String()
        vehicle_info_msg.data = json.dumps(vehicle_info)
        self.vehicle_info_pub.publish(vehicle_info_msg)
        
        self.get_logger().info(f'차량 추적 시작 정보 발행: 일반차량 TAG_{self.tag_id}')

    def move_along_route(self):
        """경로를 따라 이동"""
        self.current_target_index = 0
        
        # 각 목표점까지 이동
        for i, target in enumerate(self.route_points):
            if not self.is_running:
                break
                
            self.get_logger().info(f'목표점 {i+1}/{len(self.route_points)}로 이동: {target}')
            self.move_to_target(target[0], target[1])
            
            # 각 지점에서 잠시 대기
            if i < len(self.route_points) - 1:
                self.get_logger().info(f'목표점 {i+1} 도달, 0.8초 대기')
                time.sleep(0.8)

        # 마지막 지점에서 계속 머물러서 주차 감지되도록 함
        if self.is_running:
            self.get_logger().info('4번 주차구역에 도달, 주차 상태 유지')
            while self.is_running:
                self.publish_current_position()
                time.sleep(1.0)
                self.get_logger().debug(f'주차 중: ({self.current_position[0]:.0f}, {self.current_position[1]:.0f})')
            
            self.get_logger().info('=== 테스트 완료 ===')
            self.is_running = False

    def move_to_target(self, target_x, target_y):
        """목표 지점으로 부드럽게 이동"""
        while self.is_running:
            # 현재 위치에서 목표까지의 거리 계산
            dx = target_x - self.current_position[0]
            dy = target_y - self.current_position[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < 5.0:  # 목표에 도달
                self.current_position = [target_x, target_y]
                self.publish_current_position()
                break
            
            # 이동 방향 계산
            move_x = (dx / distance) * self.movement_speed
            move_y = (dy / distance) * self.movement_speed
            
            self.current_position[0] += move_x
            self.current_position[1] += move_y
            
            # 위치 발행
            self.publish_current_position()
            time.sleep(0.05)  # 20Hz로 발행

    def publish_current_position(self):
        """현재 위치를 UWB 좌표로 발행"""
        uwb_msg = PointStamped()
        uwb_msg.header.stamp = self.get_clock().now().to_msg()
        uwb_msg.header.frame_id = f"tag_{self.tag_id}"
        uwb_msg.point.x = float(self.current_position[0])
        uwb_msg.point.y = float(self.current_position[1])
        uwb_msg.point.z = 0.0
        
        self.uwb_comp_pub.publish(uwb_msg)
        
        # self.get_logger().debug(f'위치 발행: TAG_{self.tag_id} ({self.current_position[0]:.1f}, {self.current_position[1]:.1f})')

def main(args=None):
    rclpy.init(args=args)
    
    dummy_node = NormalVehicleTest4Dummy()
    
    # 별도 스레드에서 테스트 실행
    def run_test():
        time.sleep(2.0)  # 시스템 초기화 대기
        dummy_node.start_test()
    
    test_thread = threading.Thread(target=run_test, daemon=True)
    test_thread.start()
    
    try:
        rclpy.spin(dummy_node)
    except KeyboardInterrupt:
        dummy_node.get_logger().info('Ctrl+C로 종료 요청됨')
        dummy_node.stop_test()
    finally:
        dummy_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()