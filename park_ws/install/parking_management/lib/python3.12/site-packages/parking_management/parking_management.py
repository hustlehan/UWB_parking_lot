#GUI 경로 전송 프로그램 - TCP 통신 전용

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from geometry_msgs.msg import PointStamped
import json
import socket
from typing import List, Tuple, Optional
from datetime import datetime
import time

class ParkingManagementNode(Node):
    def __init__(self):
        super().__init__('parking_management_node')
        
        # 파라미터 설정
        self.declare_parameter('teammate_ip', '172.20.105.69')
        self.declare_parameter('teammate_port', 9999)
        
        self.teammate_ip = self.get_parameter('teammate_ip').value
        self.teammate_port = self.get_parameter('teammate_port').value
        
        # 주차장 설정
        self.init_parking_system()
        
        # ROS2 토픽 설정
        self.setup_ros_topics()
        
        # 주차공간 정보 저장
        self.current_spot_info = {}
        self.pending_requests = {}  # 배정 대기 중인 요청들
        
        self.get_logger().info('주차장 관제 노드 시작 (TCP 통신 전용)')
        self.get_logger().info(f'팀원 노트북: {self.teammate_ip}:{self.teammate_port}')
        self.get_logger().info('UWB 실시간 좌표 전송 기능 활성화')
    
    def init_parking_system(self):
        """주차장 시스템 초기화"""
        # 기본 설정
        self.MANDATORY_WAYPOINT = (200, 925)
        
        # 주차구역별 waypoint 좌표
        self.parking_waypoints = {
            # 주차구역 1-5 (상단, 왼쪽→오른쪽)
            1: (200, 1475), 2: (550, 1475), 3: (850, 1475), 4: (1150, 1475),
            5: (1450, 1475),
            # 주차구역 6-7 (우측, 위→아래)  
            6: (1475, 1400), 7: (1475, 1000),
            # 주차구역 8-11 (하단, 오른쪽→왼쪽)
            8: (1475, 925), 9: (1150, 925), 10: (850, 925), 11: (550, 925)
        }
        
        self.get_logger().info('주차장 시스템 초기화 완료')
    
    def setup_ros_topics(self):
        """ROS2 토픽 설정"""
        # Subscribers
        self.spot_sub = self.create_subscription(
            Int32, '/assign_spot', self.assign_spot_callback, 10)
        
        self.spot_info_sub = self.create_subscription(
            String, '/parking/spot_info', self.spot_info_callback, 10)
            
        self.spot_assignment_sub = self.create_subscription(
            String, '/parking/spot_assignment', self.spot_assignment_callback, 10)
        
        self.vehicle_info_sub = self.create_subscription(
            String, '/uwb/vehicle_info', self.vehicle_info_callback, 10)
        
        # UWB 실시간 좌표 구독 추가
        self.uwb_comp_sub = self.create_subscription(
            PointStamped, '/uwb/comp', self.uwb_comp_callback, 10)
        
        # Publishers
        self.spot_request_pub = self.create_publisher(
            String, '/parking/spot_request', 10)
        
        self.status_pub = self.create_publisher(String, '/parking_status', 10)
        self.waypoint_pub = self.create_publisher(String, '/waypoint_result', 10)

    def uwb_comp_callback(self, msg):
        """UWB 실시간 좌표 수신 및 팀원에게 전송"""
        try:
            frame_id = msg.header.frame_id
            if frame_id.startswith("tag_"):
                tag_id = int(frame_id[4:])
                
                # 좌표 데이터 구성
                position_data = {
                    'type': 'real_time_position',
                    'tag_id': tag_id,
                    'frame_id': frame_id,
                    'x': float(msg.point.x),
                    'y': float(msg.point.y),
                    'z': float(msg.point.z),
                    'timestamp': datetime.now().isoformat(),
                    'source': 'parking_management_node'
                }
                
                # TCP로 실시간 좌표 전송
                self.send_tcp_message(position_data, timeout=0.5)
                
            else:
                self.get_logger().warn(f'Invalid frame_id format: {frame_id}')
                
        except Exception as e:
            self.get_logger().error(f'UWB 좌표 처리 중 오류: {str(e)}')

    def vehicle_info_callback(self, msg):
        """UWB 제어 시스템으로부터 차량 정보 수신 - 자동 배정 트리거"""
        try:
            vehicle_info = json.loads(msg.data)
            action = vehicle_info.get("action")
            
            if action == "start_tracking":
                vehicle_id = vehicle_info.get("vehicle_id")
                elec = vehicle_info.get("elec", False)
                disabled = vehicle_info.get("disabled", False)
                preferred = vehicle_info.get("preferred", "normal")
                
                self.get_logger().info(f'자동 배정 시작: {vehicle_id} (preferred={preferred}, '
                                     f'elec={elec}, disabled={disabled})')
                
                # 관리자 프로그램에 주차공간 배정 요청
                self.request_parking_spot(vehicle_id, preferred, elec, disabled)
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f'차량 정보 JSON 파싱 실패: {e}')

    def request_parking_spot(self, vehicle_id: str, preferred: str, elec: bool, disabled: bool):
        """관리자 프로그램에 주차공간 배정 요청"""
        request_data = {
            "vehicle_id": vehicle_id,
            "preferred": preferred,
            "elec": elec,
            "disabled": disabled,
            "timestamp": datetime.now().isoformat()
        }
        
        # 대기 목록에 추가
        self.pending_requests[vehicle_id] = {
            "request_data": request_data,
            "request_time": time.time()
        }
        
        # 요청 발행
        request_msg = String()
        request_msg.data = json.dumps(request_data)
        self.spot_request_pub.publish(request_msg)
        
        self.get_logger().info(f'주차공간 배정 요청 전송: {vehicle_id}')

    def spot_info_callback(self, msg):
        """관리자 프로그램으로부터 주차공간 정보 수신"""
        try:
            spot_info = json.loads(msg.data)
            self.current_spot_info = spot_info
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'주차공간 정보 JSON 파싱 실패: {e}')

    def spot_assignment_callback(self, msg):
        """관리자 프로그램으로부터 주차공간 배정 결과 수신"""
        try:
            assignment_data = json.loads(msg.data)
            vehicle_id = assignment_data.get("vehicle_id")
            assigned_spot = assignment_data.get("assigned_spot")
            
            if vehicle_id not in self.pending_requests:
                self.get_logger().warn(f'예상하지 못한 배정 결과: {vehicle_id}')
                return
            
            # 대기 목록에서 제거
            del self.pending_requests[vehicle_id]
            
            if assigned_spot:
                self.get_logger().info(f'주차공간 배정 완료: {vehicle_id} -> {assigned_spot}번')
                
                # waypoint 계산 및 전송
                waypoints = self.calculate_waypoints(assigned_spot)
                if waypoints:
                    success = self.send_waypoints_to_teammate(assigned_spot, waypoints, vehicle_id)
                    self.publish_waypoint_result(assigned_spot, waypoints, success, vehicle_id)
                    
                    if success:
                        self.publish_status(f'{vehicle_id} -> {assigned_spot}번 구역 전송 성공 (자동 배정)')
                    else:
                        self.publish_status(f'{vehicle_id} -> {assigned_spot}번 구역 전송 실패')
                else:
                    self.get_logger().error(f'{assigned_spot}번 구역 waypoint 계산 실패')
            else:
                self.get_logger().error(f'주차공간 배정 실패: {vehicle_id} - 사용 가능한 공간 없음')
                self.publish_status(f'{vehicle_id} 배정 실패 - 만차')
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f'배정 결과 JSON 파싱 실패: {e}')
    
    def calculate_waypoints(self, target_spot: int) -> List[Tuple[int, int]]:
        """주차구역 번호로 waypoint 계산"""
        if target_spot not in self.parking_waypoints:
            return []
        
        target_waypoint = self.parking_waypoints[target_spot]
        waypoints = [self.MANDATORY_WAYPOINT]  # 필수 시작점 (200, 925)
        
        if target_spot == 1:  # 1번: (200, 925) -> (200,1475)
            waypoints.append(target_waypoint)
        elif target_spot in [2, 3, 4, 5]:  # 2~5번: (200, 925) -> (200, 1475) -> 최종 주차구역
            waypoints.append((200, 1475))
            waypoints.append(target_waypoint)
        elif target_spot == 6:  # 6번: (200, 925) -> (200, 1475) -> (1475, 1475) -> (1475, 1400)
            waypoints.append((200, 1475))
            waypoints.append((1475, 1475))
            waypoints.append(target_waypoint)
        elif target_spot == 7:  # 7번: (200, 925) -> (1475, 925) -> (1475, 1000)
            waypoints.append((1475, 925))
            waypoints.append(target_waypoint)
        elif target_spot in [8, 9, 10, 11]:  # 8~11번: (200, 925) -> 최종 주차구역
            waypoints.append(target_waypoint)
        
        return waypoints
    
    def assign_spot_callback(self, msg):
        """수동 주차구역 배정 요청 처리"""
        spot_number = msg.data
        
        if spot_number < 1 or spot_number > 11:
            self.get_logger().error(f'잘못된 주차구역 번호: {spot_number} (1-11만 가능)')
            return
        
        waypoints = self.calculate_waypoints(spot_number)
        
        if not waypoints:
            self.get_logger().error(f'{spot_number}번 구역 waypoint 계산 실패')
            return
        
        self.get_logger().info(f'{spot_number}번 구역 수동 배정')
        self.get_logger().info(f'계산된 waypoints: {waypoints}')
        
        # 팀원에게 전송
        success = self.send_waypoints_to_teammate(spot_number, waypoints, f"MANUAL_{spot_number:02d}")
        
        # ROS 토픽으로도 발행
        self.publish_waypoint_result(spot_number, waypoints, success, f"MANUAL_{spot_number:02d}")
        
        if success:
            self.get_logger().info('팀원에게 전송 완료 (수동)')
            self.publish_status(f'{spot_number}번 구역 전송 성공 (수동 배정)')
        else:
            self.get_logger().error('팀원에게 전송 실패 (수동)')
            self.publish_status(f'{spot_number}번 구역 전송 실패 (수동 배정)')
    
    def send_waypoints_to_teammate(self, spot_number: int, waypoints: List[Tuple[int, int]], vehicle_id: str) -> bool:
        """팀원 노트북으로 waypoint 전송 (TCP)"""
        data = {
            'type': 'waypoint_assignment',
            'vehicle_id': vehicle_id,
            'assigned_spot': spot_number,
            'waypoints': waypoints,
            'timestamp': datetime.now().isoformat(),
            'total_waypoints': len(waypoints),
            'description': self.get_route_description(spot_number),
            'source': 'parking_management_node',
            'assignment_mode': 'auto' if not vehicle_id.startswith('MANUAL') else 'manual'
        }
        
        return self.send_tcp_message(data, timeout=5.0, expect_response=True)
    
    def send_tcp_message(self, data: dict, timeout: float = 5.0, expect_response: bool = False) -> bool:
        """TCP로 메시지 전송 (공통 함수)"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(timeout)
            sock.connect((self.teammate_ip, self.teammate_port))
            
            message = json.dumps(data, ensure_ascii=False)
            sock.send(message.encode('utf-8'))
            
            if expect_response:
                response = sock.recv(1024).decode('utf-8')
                response_data = json.loads(response)
                self.get_logger().info(f'팀원 응답: {response_data.get("status", "unknown")}')
            
            sock.close()
            return True
            
        except Exception as e:
            if data.get('type') == 'real_time_position':
                # 실시간 좌표는 debug 레벨로
                self.get_logger().debug(f'TCP 전송 실패: {e}')
            else:
                self.get_logger().error(f'TCP 전송 오류: {e}')
            return False
    
    def get_route_description(self, spot_number: int) -> str:
        """경로 설명 생성"""
        spot_type = "장애인 구역" if spot_number in [1, 6, 7] else \
                   "전기차 충전 구역" if spot_number in [4, 5] else "일반 구역"
        
        if spot_number == 1:
            return f"{spot_number}번 {spot_type}: 직진 가능 경로"
        elif spot_number in [2, 3, 4, 5]:
            return f"{spot_number}번 {spot_type}: 직진 후 해당 주차구역으로"
        elif spot_number == 6:
            return f"{spot_number}번 {spot_type}: 직진 후 우회전하여 우측 상단"
        elif spot_number == 7:
            return f"{spot_number}번 {spot_type}: 우회전하여 우측 하단"
        elif spot_number in [8, 9, 10, 11]:
            return f"{spot_number}번 {spot_type}: 우회전 후 하단 구역"
        else:
            return f"{spot_number}번 {spot_type}: 경로 계산됨"
    
    def publish_waypoint_result(self, spot_number: int, waypoints: List[Tuple[int, int]], success: bool, vehicle_id: str):
        """waypoint 결과를 ROS 토픽으로 발행"""
        result_data = {
            'vehicle_id': vehicle_id,
            'spot_number': spot_number,
            'waypoints': waypoints,
            'success': success,
            'timestamp': datetime.now().isoformat(),
            'waypoint_count': len(waypoints),
            'assignment_mode': 'auto' if not vehicle_id.startswith('MANUAL') else 'manual'
        }
        
        result_msg = String()
        result_msg.data = json.dumps(result_data, ensure_ascii=False)
        self.waypoint_pub.publish(result_msg)
    
    def publish_status(self, message: str):
        """상태 메시지 발행"""
        status_data = {
            'timestamp': datetime.now().isoformat(),
            'message': message,
            'node': 'parking_management_node',
            'pending_requests': len(self.pending_requests),
            'current_spot_info': self.current_spot_info.get('available_spots', {})
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status_data, ensure_ascii=False)
        self.status_pub.publish(status_msg)
    
    def destroy_node(self):
        """노드 종료 시 정리"""
        self.get_logger().info('주차장 관제 노드를 종료합니다...')
        self.get_logger().info('노드 종료 완료')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = ParkingManagementNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C로 종료 요청됨')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()