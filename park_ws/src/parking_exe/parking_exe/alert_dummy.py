#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
import json
import time
import threading
import math

class IllegalParkingTestDummy(Node):
    def __init__(self):
        super().__init__('illegal_parking_test_dummy')
        
        # Publishers
        self.uwb_comp_pub = self.create_publisher(PointStamped, '/uwb/comp', 10)
        self.vehicle_info_pub = self.create_publisher(String, '/uwb/vehicle_info', 10)
        
        # 테스트 설정
        self.tag_id = 99
        self.vehicle_id = "TEST일반차"
        self.is_running = False
        
        # 위치 설정
        self.start_pos = (200.0, 200.0)     # 출발점
        self.target_pos = (450.0, 1800.0)   # 2번 장애인 주차구역 중간 좌표
        self.current_pos = list(self.start_pos)
        
        self.get_logger().info('=== 불법주차 테스트 더미 노드 ===')
        self.get_logger().info(f'출발점: {self.start_pos}')
        self.get_logger().info(f'목표점: {self.target_pos} (2번 장애인 주차구역 중간)')

    def start_test(self):
        """테스트 시작"""
        if self.is_running:
            return
            
        self.is_running = True
        self.get_logger().info('\n테스트 시작!')
        
        # Step 1: 차량 정보 발행
        self.send_vehicle_info()
        time.sleep(1)
        
        # Step 2: 출발점에서 시작
        self.current_pos = list(self.start_pos)
        self.get_logger().info(f'출발점에서 시작: {self.current_pos}')
        self.send_position()
        time.sleep(2)
        
        # Step 3: 목표점으로 이동
        self.get_logger().info(f'목표점으로 이동 시작: {self.start_pos} → {self.target_pos}')
        self.move_to_target()
        
        # Step 4: 목표점에서 주차 상태 유지
        self.get_logger().info('2번 장애인 주차구역에서 불법 주차 중...')
        self.park_and_wait()

    def send_vehicle_info(self):
        """일반차량 정보 발행"""
        vehicle_data = {
            "tag_id": self.tag_id,
            "vehicle_id": self.vehicle_id,
            "elec": False,
            "disabled": False,  # 일반차량 (장애인차 아님)
            "owner": "테스트사용자",
            "action": "start_tracking"
        }
        
        msg = String()
        msg.data = json.dumps(vehicle_data)
        self.vehicle_info_pub.publish(msg)
        self.get_logger().info(f'차량정보 발행: 일반차량 TAG_{self.tag_id}')

    def send_position(self):
        """현재 위치 발행"""
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"tag_{self.tag_id}"
        msg.point.x = float(self.current_pos[0])
        msg.point.y = float(self.current_pos[1])
        msg.point.z = 0.0
        
        self.uwb_comp_pub.publish(msg)

    def move_to_target(self):
        """목표점으로 직선 이동"""
        start_x, start_y = self.current_pos
        target_x, target_y = self.target_pos
        
        # 총 거리 계산
        total_distance = math.sqrt((target_x - start_x)**2 + (target_y - start_y)**2)
        self.get_logger().info(f'총 이동거리: {total_distance:.1f} 픽셀')
        
        # 이동 설정
        speed = 30.0  # 픽셀/초
        steps = 50    # 총 50단계로 나누어 이동
        
        for i in range(steps + 1):
            if not self.is_running:
                break
                
            # 진행률 계산 (0.0 ~ 1.0)
            progress = i / steps
            
            # 현재 위치 계산 (선형 보간)
            current_x = start_x + (target_x - start_x) * progress
            current_y = start_y + (target_y - start_y) * progress
            
            self.current_pos = [current_x, current_y]
            self.send_position()
            
            # 진행상황 로그 (10%마다)
            if i % 5 == 0:
                percent = int(progress * 100)
                self.get_logger().info(f'이동중... {percent}% ({current_x:.1f}, {current_y:.1f})')
            
            time.sleep(0.1)  # 0.1초 간격
        
        # 최종 위치 확정
        self.current_pos = [target_x, target_y]
        self.send_position()
        self.get_logger().info(f'목표점 도달 완료: ({target_x}, {target_y})')

    def park_and_wait(self):
        """주차 상태 유지"""
        count = 0
        while self.is_running:
            count += 1
            self.send_position()
            self.get_logger().info(f'불법주차 {count}초째 (위치: {self.current_pos[0]:.1f}, {self.current_pos[1]:.1f})')
            time.sleep(1.0)

    def stop_test(self):
        """테스트 중지"""
        self.is_running = False
        self.get_logger().info('테스트 중지됨')

def main(args=None):
    rclpy.init(args=args)
    
    node = IllegalParkingTestDummy()
    
    # 테스트 실행
    def run_test():
        time.sleep(3)  # 초기화 대기
        node.start_test()
    
    test_thread = threading.Thread(target=run_test)
    test_thread.daemon = True
    test_thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\nCtrl+C 종료 신호 받음')
        node.stop_test()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()