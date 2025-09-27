import socket
import json
import time
import threading
from datetime import datetime
import math

class DummyPositionSender:
    def __init__(self, target_ip='192.168.0.74', target_port=9999):
        self.target_ip = target_ip
        self.target_port = target_port
        self.running = False
        self.current_position = [200.0, 200.0]  # 시작점 (입구)
        self.target_position = [200.0, 200.0]
        self.tag_id = 1
        self.movement_speed = 25.0  # 픽셀/초 (5배 증가)
        
    def send_position(self, x, y):
        """실시간 위치 데이터 전송"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2.0)
            sock.connect((self.target_ip, self.target_port))
            
            position_data = {
                'type': 'real_time_position',
                'tag_id': self.tag_id,
                'x': float(x),
                'y': float(y),
                'timestamp': datetime.now().isoformat(),
                'source': 'dummy_test_sender'
            }
            
            message = json.dumps(position_data, ensure_ascii=False)
            sock.send(message.encode('utf-8'))
            
            # 응답 수신
            response = sock.recv(1024).decode('utf-8')
            response_data = json.loads(response)
            print(f"위치 전송 완료: ({x:.1f}, {y:.1f}) - 응답: {response_data.get('status', 'unknown')}")
            
            sock.close()
            return True
            
        except Exception as e:
            print(f"위치 전송 실패: {e}")
            return False
    
    def send_waypoints(self, waypoints):
        """웨이포인트 데이터 전송"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5.0)
            sock.connect((self.target_ip, self.target_port))
            
            waypoint_data = {
                'type': 'waypoint_assignment',
                'vehicle_id': f'TEST_{self.tag_id:02d}',
                'assigned_spot': 2,  # 2번 주차구역으로 변경
                'waypoints': waypoints,
                'timestamp': datetime.now().isoformat(),
                'source': 'dummy_test_sender'
            }
            
            message = json.dumps(waypoint_data, ensure_ascii=False)
            sock.send(message.encode('utf-8'))
            
            response = sock.recv(1024).decode('utf-8')
            response_data = json.loads(response)
            print(f"웨이포인트 전송 완료: {waypoints} - 응답: {response_data.get('status', 'unknown')}")
            
            sock.close()
            return True
            
        except Exception as e:
            print(f"웨이포인트 전송 실패: {e}")
            return False
    
    def move_to_target(self, target_x, target_y):
        """목표 지점으로 부드럽게 이동"""
        self.target_position = [target_x, target_y]
        
        while self.running:
            # 현재 위치에서 목표까지의 거리 계산
            dx = self.target_position[0] - self.current_position[0]
            dy = self.target_position[1] - self.current_position[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < 5.0:  # 목표에 도달
                self.current_position = self.target_position.copy()
                break
            
            # 이동 방향 계산
            move_x = (dx / distance) * self.movement_speed
            move_y = (dy / distance) * self.movement_speed
            
            self.current_position[0] += move_x
            self.current_position[1] += move_y
            
            # 위치 전송
            self.send_position(self.current_position[0], self.current_position[1])
            time.sleep(0.05)  # 20Hz로 전송 (기존 5Hz에서 4배 증가)
    
    def circular_movement(self, center_x=1000, center_y=1000, radius=300):
        """원형 움직임 테스트"""
        angle = 0
        while self.running:
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            
            self.current_position = [x, y]
            self.send_position(x, y)
            
            angle += 0.1
            time.sleep(0.05)
    
    def predefined_route(self):
        """2번 주차구역의 중앙 감지구역으로 가는 미리 정의된 경로"""
        # 2번 주차구역: (300,1600) ~ (600,2000) -> 중심: (450, 1800)
        # 중앙 감지구역의 정확한 중심: (450, 1800)
        route_points = [
            (200, 200),    # 시작점 (입구)
            (200, 925),    # 필수 경유점
            (200, 1476),   # 상단으로 직진 후 정지
            (450, 1476),   # 우회전하여 2번 주차구역 앞
            (450, 1800),   # 2번 주차구역 중앙 감지구역 중심 (최종 목적지)
        ]
        
        for target_x, target_y in route_points:
            if not self.running:
                break
            print(f"목표 지점으로 이동: ({target_x}, {target_y})")
            self.move_to_target(target_x, target_y)
            time.sleep(0.5)  # 각 지점에서 잠시 정지
        
        print("2번 주차구역 중앙 감지구역에 도착했습니다!")
        
        # 목적지에서 계속 위치 전송하며 대기
        while self.running:
            self.send_position(self.current_position[0], self.current_position[1])
            time.sleep(1.0)  # 1초마다 현재 위치 전송
            print(f"주차 중: ({self.current_position[0]:.0f}, {self.current_position[1]:.0f})")
        
        for target_x, target_y in route_points:
            if not self.running:
                break
            print(f"목표 지점으로 이동: ({target_x}, {target_y})")
            self.move_to_target(target_x, target_y)
            time.sleep(0.5)  # 각 지점에서 잠시 정지 (2초에서 0.5초로 단축)
        
        print("4번 주차구역에 도착했습니다!")
    
    def start_movement_test(self, test_type='route'):
        """움직임 테스트 시작"""
        self.running = True
        
        if test_type == 'route':
            print("2번 주차구역 경로 테스트 시작")
            self.predefined_route()
        elif test_type == 'circle':
            print("원형 움직임 테스트 시작")
            self.circular_movement()
        
        self.running = False
        print("움직임 테스트 완료")
    
    def stop(self):
        """테스트 중지"""
        self.running = False

def main():
    print("=== x,y 좌표 송신 테스트 프로그램 (4번 주차구역) ===")
    print("팀원의 수신 프로그램이 실행되고 있는지 확인하세요.")
    
    # 연결 설정 (필요시 IP 주소 변경)
    sender = DummyPositionSender(target_ip='192.168.0.74', target_port=9999)
    
    while True:
        print("\n테스트 옵션:")
        print("1. 단일 위치 전송")
        print("2. 2번 주차구역 웨이포인트 전송")
        print("3. 2번 주차구역으로 경로 이동")
        print("4. 원형 움직임")
        print("5. 종료")
        
        choice = input("선택하세요 (1-5): ").strip()
        
        if choice == '1':
            try:
                x = float(input("X 좌표 입력: "))
                y = float(input("Y 좌표 입력: "))
                sender.send_position(x, y)
            except ValueError:
                print("잘못된 좌표 입력")
        
        elif choice == '2':
            # 2번 주차구역으로 가는 웨이포인트 (직진)
            test_waypoints = [
                [200, 925],    # 필수 경유점
                [450, 1475]    # 2번 주차구역
            ]
            sender.send_waypoints(test_waypoints)
        
        elif choice == '3':
            print("2번 주차구역 경로 이동 테스트를 시작합니다. Ctrl+C로 중지할 수 있습니다.")
            try:
                movement_thread = threading.Thread(target=sender.start_movement_test, args=('route',))
                movement_thread.daemon = True
                movement_thread.start()
                movement_thread.join()
            except KeyboardInterrupt:
                sender.stop()
                print("\n이동 테스트가 중지되었습니다.")
        
        elif choice == '4':
            print("원형 움직임 테스트를 시작합니다. Ctrl+C로 중지할 수 있습니다.")
            try:
                movement_thread = threading.Thread(target=sender.start_movement_test, args=('circle',))
                movement_thread.daemon = True
                movement_thread.start()
                movement_thread.join()
            except KeyboardInterrupt:
                sender.stop()
                print("\n원형 움직임 테스트가 중지되었습니다.")
        
        elif choice == '5':
            sender.stop()
            print("프로그램을 종료합니다.")
            break
        
        else:
            print("잘못된 선택입니다.")

if __name__ == '__main__':
    main()