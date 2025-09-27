#!/usr/bin/env python3

import sys
import json
import time
import threading
import socket
from typing import Dict, List, Tuple, Optional
from datetime import datetime, timedelta
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QLabel, QFrame, QPushButton, QTextEdit,
                             QGridLayout, QScrollArea, QMessageBox)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QThread
from PyQt5.QtGui import QPainter, QPen, QBrush, QColor, QFont

class StopperController:
    """ESP32 스토퍼 제어 클래스"""
    def __init__(self, host='192.168.0.90', port=8888):
        self.host = host
        self.port = port
        self.client_socket = None
        self.connected = False
        self.last_reconnect_attempt = 0
        self.reconnect_interval = 5000  # 5초
        
    def connect(self):
        """ESP32 스토퍼에 연결"""
        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.settimeout(3.0)
            self.client_socket.connect((self.host, self.port))
            self.connected = True
            print(f"[STOPPER] Connected to {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"[STOPPER] Connection failed: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """연결 해제"""
        if self.client_socket:
            self.client_socket.close()
            self.client_socket = None
        self.connected = False
    
    def send_command(self, command):
        """스토퍼에 명령 전송 (0=STOP, 1=FORWARD, 2=BACKWARD)"""
        if not self.connected:
            if not self.connect():
                return False
        
        try:
            message = f"CMD:{command}\n"
            self.client_socket.send(message.encode('utf-8'))
            
            if command == 0:
                print(f"[STOPPER] Sent: STOP command")
            elif command == 1:
                print(f"[STOPPER] Sent: FORWARD command")
            elif command == 2:
                print(f"[STOPPER] Sent: BACKWARD command")
            
            return True
        except Exception as e:
            print(f"[STOPPER] Send error: {e}")
            self.disconnect()
            return False
    
    def move_backward(self):
        """후진 명령"""
        return self.send_command(2)
    
    def move_forward(self):
        """전진 명령"""
        return self.send_command(1)
    
    def stop(self):
        """정지 명령"""
        return self.send_command(0)

@dataclass
class Vehicle:
    """차량 정보 클래스"""
    id: str
    tag_id: int  # UWB tag_id 추가
    current_position: Tuple[float, float]
    entry_time: datetime
    last_update: datetime
    is_parked: bool = False
    parked_spot: Optional[int] = None
    parking_start_time: Optional[datetime] = None
    # 차량 타입 정보 추가
    elec: bool = False      # 전기차 여부
    disabled: bool = False  # 장애인 차량 여부
    owner: str = "Unknown"  # 소유자
    # 위치 필터링을 위한 추가 변수들
    position_history: List[Tuple[float, float]] = None  # 최근 위치 이력
    smoothed_position: Tuple[float, float] = None       # 스무딩된 위치
    
    def __post_init__(self):
        if self.position_history is None:
            self.position_history = []
        if self.smoothed_position is None:
            self.smoothed_position = self.current_position

class ParkingExeNode(Node):
    """ROS2 노드 클래스"""

    def __init__(self, gui_callback, illegal_parking_callback):
        super().__init__('parking_exe_node')

        # GUI 콜백 함수
        self.gui_callback = gui_callback
        self.illegal_parking_callback = illegal_parking_callback # 불법 주차 콜백 추가

        # UWB 처리된 좌표 구독 (/uwb/comp로 변경)
        self.uwb_sub = self.create_subscription(
            PointStamped, '/uwb/comp', self.uwb_callback, 10)

        # 차량 타입 정보 구독 (새로 추가)
        self.vehicle_info_sub = self.create_subscription(
            String, '/uwb/vehicle_info', self.vehicle_info_callback, 10)

        # 주차공간 배정 요청 구독 (경로 전송 프로그램으로부터)
        self.spot_request_sub = self.create_subscription(
            String, '/parking/spot_request', self.spot_request_callback, 10)

        # 상태 발행
        self.status_pub = self.create_publisher(String, '/parking_exe/status', 10)
        
        # 주차공간 정보 발행 (경로 전송 프로그램으로)
        self.spot_info_pub = self.create_publisher(String, '/parking/spot_info', 10)
        
        # 주차공간 배정 결과 발행
        self.spot_assignment_pub = self.create_publisher(String, '/parking/spot_assignment', 10)

        # 차량 관리 (tag_id를 키로 사용)
        self.vehicles: Dict[int, Vehicle] = {}  # tag_id: Vehicle

        # 주차구역 및 중앙 감지 구역 정의
        self.parking_spots = self.define_parking_spots()

        # 스토퍼 제어기 초기화
        self.stopper_controller = StopperController()

        # 주차 감지를 위한 타이머
        self.parking_check_timer = self.create_timer(1.0, self.check_parking_status)
        
        # 주차공간 정보 발행 타이머 (5초마다)
        self.spot_info_timer = self.create_timer(5.0, self.publish_spot_info)

        self.get_logger().info('🚗 주차장 관리자 시스템 시작 (불법 주차 감지 기능 활성)')
        self.get_logger().info(f'📊 총 주차구역: {len(self.parking_spots)}개')

    def vehicle_info_callback(self, msg):
        """차량 타입 정보 수신 콜백"""
        try:
            vehicle_info = json.loads(msg.data)
            tag_id = vehicle_info.get("tag_id")
            action = vehicle_info.get("action")

            if action == "start_tracking":
                if not hasattr(self, 'pending_vehicle_info'):
                    self.pending_vehicle_info = {}
                self.pending_vehicle_info[tag_id] = {
                    "vehicle_id": vehicle_info.get("vehicle_id"),
                    "elec": vehicle_info.get("elec", False),
                    "disabled": vehicle_info.get("disabled", False),
                    "owner": vehicle_info.get("owner", "Unknown")
                }
                self.get_logger().info(f'📋 차량 정보 수신: TAG_{tag_id} - '
                                     f'전기차={vehicle_info.get("elec")}, '
                                     f'장애인={vehicle_info.get("disabled")}')
            elif action == "stop_tracking":
                if tag_id in self.vehicles:
                    del self.vehicles[tag_id]
                    self.get_logger().info(f'🚙 차량 출차 (추적 종료): TAG_{tag_id}')
        except json.JSONDecodeError as e:
            self.get_logger().error(f'차량 정보 JSON 파싱 실패: {e}')

    def spot_request_callback(self, msg):
        """주차공간 배정 요청 처리 콜백 (경로 전송 프로그램으로부터)"""
        try:
            request_data = json.loads(msg.data)
            vehicle_id = request_data.get("vehicle_id")
            preferred = request_data.get("preferred", "normal")
            elec = request_data.get("elec", False)
            disabled = request_data.get("disabled", False)
            
            self.get_logger().info(f'🎯 주차공간 배정 요청: {vehicle_id}, preferred={preferred}, '
                                 f'elec={elec}, disabled={disabled}')
            
            # 주차공간 자동 배정
            assigned_spot = self.assign_parking_spot(preferred, elec, disabled)
            
            # 배정 결과 발행
            assignment_result = {
                "vehicle_id": vehicle_id,
                "assigned_spot": assigned_spot,
                "preferred": preferred,
                "elec": elec,
                "disabled": disabled,
                "timestamp": datetime.now().isoformat()
            }
            
            assignment_msg = String()
            assignment_msg.data = json.dumps(assignment_result)
            self.spot_assignment_pub.publish(assignment_msg)
            
            if assigned_spot:
                spot_type = self.get_spot_type_name(assigned_spot)
                self.get_logger().info(f'✅ 주차공간 배정 완료: {vehicle_id} -> {assigned_spot}번 ({spot_type})')
                
                # 장애인 구역 배정 시 스토퍼 후진 명령
                if assigned_spot in [1, 6, 7]:
                    self.get_logger().info(f'🚧 장애인 구역 배정 -> 스토퍼 후진 명령 전송')
                    threading.Thread(target=self.control_stopper_backward, daemon=True).start()
                    
            else:
                self.get_logger().warn(f'❌ 주차공간 배정 실패: {vehicle_id} - 사용 가능한 공간 없음')
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f'주차공간 요청 JSON 파싱 실패: {e}')

    def assign_parking_spot(self, preferred: str, elec: bool, disabled: bool) -> Optional[int]:
        """주차공간 자동 배정 로직"""
        # 현재 점유 상태 확인
        occupied_spots = set()
        for vehicle in self.vehicles.values():
            if vehicle.is_parked and vehicle.parked_spot:
                occupied_spots.add(vehicle.parked_spot)
        
        # 구역별 사용 가능한 공간 확인
        disabled_spots = [1, 6, 7]  # 장애인 구역
        elec_spots = [4, 5]         # 전기차 충전 구역
        general_spots = [2, 3, 8, 9, 10, 11]  # 일반 구역
        
        available_disabled = [spot for spot in disabled_spots if spot not in occupied_spots]
        available_elec = [spot for spot in elec_spots if spot not in occupied_spots]
        available_general = [spot for spot in general_spots if spot not in occupied_spots]
        
        self.get_logger().info(f'🅿️ 사용 가능한 공간 - 장애인: {len(available_disabled)}개, '
                             f'전기차: {len(available_elec)}개, 일반: {len(available_general)}개')
        
        # 배정 로직 - preferred 값에 따른 사용자 선호도 존중
        if preferred == "disabled":
            # 1. preferred값이 장애인 주차구역
            if available_disabled:
                return available_disabled[0]
            elif available_general:
                self.get_logger().info('⚠️ 장애인 구역 만석, 일반 구역으로 대체 배정')
                return available_general[0]
            else:
                return None
                
        elif preferred == "elec":
            # 2. preferred값이 전기차 충전구역
            if available_elec:
                return available_elec[0]
            else:
                # 충전 구역이 만석인 경우 - 차량 속성에 따른 대체 배정
                if disabled:
                    # 장애인 차량이면 장애인 구역을 다음 우선순위로 고려
                    if available_disabled:
                        self.get_logger().info('⚠️ 충전 구역 만석, 장애인 차량이므로 장애인 구역으로 대체 배정')
                        return available_disabled[0]
                    elif available_general:
                        self.get_logger().info('⚠️ 충전/장애인 구역 모두 만석, 일반 구역으로 대체 배정')
                        return available_general[0]
                    else:
                        return None
                else:
                    # 비장애인 차량이면 바로 일반 구역으로
                    if available_general:
                        self.get_logger().info('⚠️ 충전 구역 만석, 일반 구역으로 대체 배정')
                        return available_general[0]
                    else:
                        return None
                    
        else:  # preferred == "normal" or 기타
            # 3. preferred값이 일반구역 (사용자가 직접 선택한 선호도이므로 존중)
            if available_general:
                return available_general[0]
            else:
                return None

    def get_spot_type_name(self, spot_id: int) -> str:
        """주차구역 타입 이름 반환"""
        if spot_id in [1, 6, 7]:
            return "장애인 구역"
        elif spot_id in [4, 5]:
            return "전기차 충전 구역"
        else:
            return "일반 구역"

    def publish_spot_info(self):
        """주차공간 정보를 주기적으로 발행"""
        # 현재 점유 상태 확인
        occupied_spots = set()
        for vehicle in self.vehicles.values():
            if vehicle.is_parked and vehicle.parked_spot:
                occupied_spots.add(vehicle.parked_spot)
        
        # 구역별 사용 가능한 공간 계산
        disabled_spots = [1, 6, 7]
        elec_spots = [4, 5]
        general_spots = [2, 3, 8, 9, 10, 11]
        
        available_disabled = len([spot for spot in disabled_spots if spot not in occupied_spots])
        available_elec = len([spot for spot in elec_spots if spot not in occupied_spots])
        available_general = len([spot for spot in general_spots if spot not in occupied_spots])
        
        # 주차공간 정보 구성
        spot_info = {
            "timestamp": datetime.now().isoformat(),
            "total_vehicles": len(self.vehicles),
            "parked_vehicles": sum(1 for v in self.vehicles.values() if v.is_parked),
            "available_spots": {
                "disabled": available_disabled,
                "elec": available_elec,
                "general": available_general
            },
            "occupied_spots": list(occupied_spots),
            "total_spots": {
                "disabled": len(disabled_spots),
                "elec": len(elec_spots), 
                "general": len(general_spots)
            }
        }
        
        # 발행
        spot_msg = String()
        spot_msg.data = json.dumps(spot_info)
        self.spot_info_pub.publish(spot_msg)

    def define_parking_spots(self) -> Dict[int, Dict[str, float]]:
        """주차구역 및 중앙 200x200 감지 구역 정의"""
        spots = {}
        detection_zone_size = 200.0

        spot_coords = {
            1: [(0,1600), (400,1600), (0,2000), (400,2000)],
            2: [(400,1600), (700,1600), (400,2000), (700,2000)],
            3: [(700,1600), (1000,1600), (700,2000), (1000,2000)],
            4: [(1000,1600), (1300,1600), (1000,2000), (1300,2000)],
            5: [(1300,1600), (1600,1600), (1300,2000), (1600,2000)],
            6: [(1600,1600), (1600,1200), (2000,1600), (2000,1200)],
            7: [(1600,1200), (1600,800), (2000,1200), (2000,800)],
            8: [(1600,800), (1300,800), (1600,400), (1300,400)],
            9: [(1300,800), (1000,800), (1300,400), (1000,400)],
            10: [(1000,800), (700,800), (1000,400), (700,400)],
            11: [(700,800), (400,800), (700,400), (400,400)]
        }

        for spot_id, coords in spot_coords.items():
            min_x = min(c[0] for c in coords); max_x = max(c[0] for c in coords)
            min_y = min(c[1] for c in coords); max_y = max(c[1] for c in coords)
            center_x = min_x + (max_x - min_x) / 2; center_y = min_y + (max_y - min_y) / 2
            half_size = detection_zone_size / 2
            spots[spot_id] = {
                'min_x': min_x, 'max_x': max_x, 'min_y': min_y, 'max_y': max_y, 'coords': coords,
                'inner_min_x': center_x - half_size, 'inner_max_x': center_x + half_size,
                'inner_min_y': center_y - half_size, 'inner_max_y': center_y + half_size,
            }
        return spots

    def get_parking_spot(self, x: float, y: float) -> Optional[int]:
        """해당 좌표가 어느 주차구역의 *중앙 감지 구역*에 속하는지 확인"""
        for spot_id, spot_info in self.parking_spots.items():
            if (spot_info['inner_min_x'] <= x <= spot_info['inner_max_x'] and
                spot_info['inner_min_y'] <= y <= spot_info['inner_max_y']):
                return spot_id
        return None

    def uwb_callback(self, msg):
        frame_id = msg.header.frame_id
        if not frame_id.startswith("tag_"): return
        try: tag_id = int(frame_id[4:])
        except ValueError: return
        x, y = msg.point.x, msg.point.y
        self.update_or_create_vehicle(tag_id, x, y, datetime.now())
        self.gui_callback(self.get_system_status())

    def update_or_create_vehicle(self, tag_id: int, x: float, y: float, current_time: datetime):
        if tag_id in self.vehicles:
            vehicle = self.vehicles[tag_id]
            
            # 위치 필터링 적용
            filtered_position = self.apply_position_filter(vehicle, x, y)
            
            vehicle.current_position = filtered_position
            vehicle.last_update = current_time
        else:
            vehicle_info = getattr(self, 'pending_vehicle_info', {}).get(tag_id, {})
            self.vehicles[tag_id] = Vehicle(
                id=f"TAG_{tag_id}", tag_id=tag_id, current_position=(x, y),
                entry_time=current_time, last_update=current_time,
                elec=vehicle_info.get("elec", False), disabled=vehicle_info.get("disabled", False),
                owner=vehicle_info.get("owner", "Unknown")
            )
            # 새 차량의 경우 초기 위치 설정
            vehicle = self.vehicles[tag_id]
            vehicle.position_history = [(x, y)]
            vehicle.smoothed_position = (x, y)
            
            self.get_logger().info(f'🚗 새 차량 추적 시작: TAG_{tag_id}')
            if hasattr(self, 'pending_vehicle_info') and tag_id in self.pending_vehicle_info:
                del self.pending_vehicle_info[tag_id]

    def apply_position_filter(self, vehicle: Vehicle, new_x: float, new_y: float) -> Tuple[float, float]:
        """위치 필터링을 적용하여 노이즈 제거"""
        
        # 1. 거리 기반 이상치 감지 (급격한 움직임 감지)
        current_x, current_y = vehicle.smoothed_position
        distance = ((new_x - current_x) ** 2 + (new_y - current_y) ** 2) ** 0.5
        
        # 임계값 설정: 모형차 기준 1초에 1,400mm 이상 이동하면 이상치로 판단 (1mm = 1픽셀)
        max_movement_per_sec = 1400.0  # 1.4m/s = 5km/h (모형차 합리적 최대 속도)
        
        if distance > max_movement_per_sec:
            # 이상치 감지 시 이전 위치 유지
            self.get_logger().warn(f'TAG_{vehicle.tag_id}: 급격한 위치 변화 감지 ({distance:.1f}cm), 필터링 적용')
            return vehicle.smoothed_position
        
        # 2. 이동 평균 필터 적용
        vehicle.position_history.append((new_x, new_y))
        
        # 최근 5개 위치만 유지
        if len(vehicle.position_history) > 5:
            vehicle.position_history = vehicle.position_history[-5:]
        
        # 가중 이동 평균 계산 (최근 데이터에 더 높은 가중치)
        weights = [0.1, 0.15, 0.2, 0.25, 0.3]  # 총합 = 1.0
        if len(vehicle.position_history) < 5:
            # 데이터가 부족한 경우 균등 가중치 사용
            weight_sum = 1.0 / len(vehicle.position_history)
            avg_x = sum(pos[0] for pos in vehicle.position_history) * weight_sum
            avg_y = sum(pos[1] for pos in vehicle.position_history) * weight_sum
        else:
            # 가중 평균 계산
            avg_x = sum(pos[0] * weight for pos, weight in zip(vehicle.position_history, weights))
            avg_y = sum(pos[1] * weight for pos, weight in zip(vehicle.position_history, weights))
        
        # 3. 추가적인 스무딩 (이전 스무딩된 위치와의 보간)
        smoothing_factor = 0.7  # 새 위치에 70% 가중치, 이전 위치에 30% 가중치
        final_x = avg_x * smoothing_factor + current_x * (1 - smoothing_factor)
        final_y = avg_y * smoothing_factor + current_y * (1 - smoothing_factor)
        
        # 스무딩된 위치 업데이트
        vehicle.smoothed_position = (final_x, final_y)
        
        return (final_x, final_y)

    def check_parking_status(self):
        """주차 상태 및 불법 주차 확인 (1초마다 실행)"""
        current_time = datetime.now()
        for tag_id, vehicle in list(self.vehicles.items()):
            if (current_time - vehicle.last_update).seconds > 10:
                self.get_logger().info(f'🚙 차량 출차 (10초 이상 신호 없음): TAG_{tag_id}')
                del self.vehicles[tag_id]
                continue

            x, y = vehicle.current_position
            current_spot = self.get_parking_spot(x, y)

            if current_spot:
                if vehicle.parking_start_time is None:
                    vehicle.parking_start_time = current_time
                    vehicle.parked_spot = current_spot
                    self.get_logger().info(f'🅿️  차량 TAG_{tag_id}이 {current_spot}번 감지 구역 진입')

                elif (current_time - vehicle.parking_start_time).seconds >= 5:
                    if not vehicle.is_parked:
                        vehicle.is_parked = True
                        self.get_logger().info(f'✅ 차량 TAG_{tag_id}이 {current_spot}번 구역에 주차 완료')

                        # --- 불법 주차 감지 로직 ---
                        is_illegal = False
                        # 장애인 구역(1,6,7)에 비장애인 차량이 주차
                        if current_spot in [1, 6, 7] and not vehicle.disabled:
                            is_illegal = True
                        # 전기차 구역(4,5)에 비전기차 차량이 주차
                        elif current_spot in [4, 5] and not vehicle.elec:
                            is_illegal = True

                        if is_illegal:
                            self.get_logger().warn(f'🚨 불법 주차 감지: TAG_{tag_id} -> {current_spot}번 구역')
                            # GUI에 팝업 요청 콜백 호출
                            self.illegal_parking_callback(vehicle.tag_id, current_spot)

            elif not current_spot and vehicle.parking_start_time:
                vehicle.parking_start_time = None
                if vehicle.is_parked:
                    vehicle.is_parked = False
                    previous_spot = vehicle.parked_spot
                    self.get_logger().info(f'🚗 차량 TAG_{tag_id}이 {previous_spot}번 감지 구역에서 벗어남')
                    
                    # 장애인 구역에서 출차 시 스토퍼 전진 명령
                    if previous_spot in [1, 6, 7]:
                        self.get_logger().info(f'🚧 장애인 구역 출차 -> 스토퍼 전진 명령 전송')
                        threading.Thread(target=self.control_stopper_forward, daemon=True).start()
                        
                vehicle.parked_spot = None

    def get_system_status(self) -> dict:
        return {
            'vehicles': self.vehicles,
            'total_vehicles': len(self.vehicles),
            'parked_vehicles': sum(1 for v in self.vehicles.values() if v.is_parked),
            'parking_spots': self.parking_spots,
        }

    def control_stopper_backward(self):
        """스토퍼 후진 명령 (별도 스레드에서 실행)"""
        try:
            success = self.stopper_controller.move_backward()
            if success:
                self.get_logger().info('✅ 스토퍼 후진 명령 전송 성공')
            else:
                self.get_logger().error('❌ 스토퍼 후진 명령 전송 실패')
        except Exception as e:
            self.get_logger().error(f'스토퍼 후진 제어 오류: {str(e)}')

    def control_stopper_forward(self):
        """스토퍼 전진 명령 (별도 스레드에서 실행)"""
        try:
            success = self.stopper_controller.move_forward()
            if success:
                self.get_logger().info('✅ 스토퍼 전진 명령 전송 성공')
            else:
                self.get_logger().error('❌ 스토퍼 전진 명령 전송 실패')
        except Exception as e:
            self.get_logger().error(f'스토퍼 전진 제어 오류: {str(e)}')

class ParkingVisualizationWidget(QWidget):
    """주차장 시각화 위젯"""
    def __init__(self):
        super().__init__()
        self.system_status = {'vehicles': {}, 'parking_spots': {}}
        self.setFixedSize(1000, 1000)
        self.scale_factor = 1000 / 2000
        
    def update_status(self, status):
        self.system_status = status
        self.update()
        
    def paintEvent(self, event):
        painter = QPainter(self)
        self.draw_parking_lot(painter)
        
    def draw_parking_lot(self, painter):
        painter.fillRect(0, 0, 1000, 1000, QColor(240, 240, 240))
        
        # 여백을 추가한 변환 함수 - 상하좌우 50픽셀 여백
        margin = 50
        available_size = 1000 - (2 * margin)
        scale_factor = available_size / 2000
        def tf(x, y): 
            return int(x * scale_factor + margin), int(1000 - margin - (y * scale_factor))
        
        # 입구 라벨 추가
        self.draw_entrance_labels(painter)
        
        self.draw_parking_spots_fixed(painter, tf)
        self.draw_vehicles_fixed(painter, tf)
        self.draw_forbidden_zone_fixed(painter, tf)

    def draw_entrance_labels(self, painter):
        """입구 라벨을 그리는 메서드"""
        painter.setPen(QPen(Qt.black, 2))
        painter.setFont(QFont('Arial', 14, QFont.Bold))
        
        # 여백을 추가한 변환 함수 - 상하좌우 50픽셀 여백
        margin = 50
        available_size = 1000 - (2 * margin)
        scale_factor = available_size / 2000
        def tf(x, y): 
            return int(x * scale_factor + margin), int(1000 - margin - (y * scale_factor))
        
        painter.drawText(20, 30, "백화점 본관 입구")
        
        movie_x, movie_y = tf(1700, 1800)
        painter.drawText(movie_x, movie_y, "영화관 입구")
        
        culture_x, culture_y = tf(1700, 600)
        painter.drawText(culture_x, culture_y, "문화시설 입구")

    def draw_parking_spots_fixed(self, painter, tf):
        for spot_id, spot in self.system_status.get('parking_spots', {}).items():
            is_occupied = any(v.is_parked and v.parked_spot == spot_id for v in self.system_status.get('vehicles', {}).values())
            x1, y1 = tf(spot['min_x'], spot['min_y']); x2, y2 = tf(spot['max_x'], spot['max_y'])
            w, h = x2 - x1, y1 - y2
            if not (w > 0 and h > 0): continue
            if spot_id in [1, 6, 7]: color = QColor(135, 206, 250) if not is_occupied else QColor(100, 150, 255)
            elif spot_id in [4, 5]: color = QColor(144, 238, 144) if not is_occupied else QColor(80, 200, 80)
            else: color = QColor(245, 245, 245) if not is_occupied else QColor(180, 180, 180)
            painter.fillRect(x1, y2, w, h, color); painter.setPen(QPen(Qt.black, 2)); painter.drawRect(x1, y2, w, h)
            ix1, iy1 = tf(spot['inner_min_x'], spot['inner_min_y']); ix2, iy2 = tf(spot['inner_max_x'], spot['inner_max_y'])
            iw, ih = ix2 - ix1, iy1 - iy2
            painter.setPen(QPen(QColor(100, 100, 100), 1, Qt.DotLine)); painter.setBrush(QBrush(QColor(0, 0, 0, 15))); painter.drawRect(ix1, iy2, iw, ih)
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            painter.setPen(QPen(Qt.black, 1)); painter.setFont(QFont('Arial', 30)); painter.drawText(cx - 15, cy + 10, str(spot_id))
            if spot_id in [1, 6, 7]: painter.setFont(QFont('Arial', 40)); painter.drawText(cx - 20, cy + 50, "♿")
            elif spot_id in [4, 5]: painter.setFont(QFont('Arial', 20)); painter.drawText(cx - 15, y1 - 10, "EV")

    def draw_vehicles_fixed(self, painter, tf):
        for v in self.system_status.get('vehicles', {}).values():
            px, py = tf(v.current_position[0], v.current_position[1])
            if v.elec and v.disabled: color = QColor(135, 206, 235)
            elif v.elec: color = QColor(144, 238, 144)
            elif v.disabled: color = QColor(0, 0, 255)
            else: color = QColor(255, 255, 255)
            border_color, border_width = (QColor(0, 100, 0), 3) if v.is_parked else (QColor(255, 140, 0), 2)
            radius = 25
            painter.setPen(QPen(border_color, border_width)); painter.setBrush(QBrush(color)); painter.drawEllipse(px - radius, py - radius, radius * 2, radius * 2)
            tag_display = str(v.tag_id)
            painter.setPen(QPen(Qt.black, 1)); painter.setFont(QFont('Arial', 12, QFont.Bold)); painter.drawText(px - (len(tag_display) * 6)//2, py + 5, tag_display)

    def draw_forbidden_zone_fixed(self, painter, tf):
        x1, y1 = tf(550, 1050); x2, y2 = tf(1350, 1350); w, h = x2 - x1, y1 - y2
        painter.setPen(QPen(Qt.red, 2)); painter.setBrush(QBrush(QColor(255, 0, 0, 100))); painter.drawRect(x1, y2, w, h)
        painter.setPen(QPen(Qt.red, 1)); painter.setFont(QFont('Arial', 20)); painter.drawText((x1 + x2) // 2 - 40, (y1 + y2) // 2 + 10, "금지구역")

class ParkingExeMainWindow(QMainWindow):
    """메인 윈도우"""
    update_signal = pyqtSignal(dict)
    illegal_parking_signal = pyqtSignal(str) # 불법 주차 알림용 시그널 추가

    def __init__(self):
        super().__init__()
        self.setWindowTitle('🚗 주차장 관리자 시스템 (UWB TAG 기반)')
        self.setGeometry(100, 100, 1300, 1000)  # 전체 창 크기 축소
        central_widget = QWidget(); self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        self.visualization = ParkingVisualizationWidget()
        main_layout.addWidget(self.visualization, 3)  # 시각화 부분을 더 크게 (3:1 비율)
        info_panel = self.create_info_panel()
        main_layout.addWidget(info_panel, 1)

        # 시그널 연결
        self.update_signal.connect(self.update_display)
        self.illegal_parking_signal.connect(self.show_illegal_parking_popup)

        self.ros_thread = QThread(); self.ros_node = None

    def create_info_panel(self):
        panel = QFrame(); panel.setFrameStyle(QFrame.Box); layout = QVBoxLayout(panel)
        title = QLabel('📊 주차장 현황 (TAG 기반)'); title.setFont(QFont('Arial', 16, QFont.Bold)); layout.addWidget(title)
        self.status_labels = {
            'total_vehicles': QLabel('총 차량: 0대'), 'parked_vehicles': QLabel('주차 완료: 0대'),
            'available_disabled': QLabel('잔여 장애인: 3대'), 'available_ev': QLabel('잔여 EV충전: 2대'),
            'available_general': QLabel('잔여 일반: 6대')
        }
        for label in self.status_labels.values(): label.setFont(QFont('Arial', 12)); layout.addWidget(label)
        layout.addWidget(QLabel())
        vehicles_title = QLabel('🏷️ TAG 차량 목록'); vehicles_title.setFont(QFont('Arial', 14, QFont.Bold)); layout.addWidget(vehicles_title)
        self.vehicles_text = QTextEdit(); self.vehicles_text.setMaximumHeight(200); layout.addWidget(self.vehicles_text)
        layout.addStretch()
        return panel

    def update_display(self, status):
        self.visualization.update_status(status)
        vehicles = status.get('vehicles', {}).values()
        disabled_occupied = sum(1 for v in vehicles if v.is_parked and v.parked_spot in [1, 6, 7])
        ev_occupied = sum(1 for v in vehicles if v.is_parked and v.parked_spot in [4, 5])
        self.status_labels['total_vehicles'].setText(f"총 차량: {status.get('total_vehicles', 0)}대")
        self.status_labels['parked_vehicles'].setText(f"주차 완료: {status.get('parked_vehicles', 0)}대")
        self.status_labels['available_disabled'].setText(f'잔여 장애인: {3 - disabled_occupied}대')
        self.status_labels['available_ev'].setText(f'잔여 EV충전: {2 - ev_occupied}대')
        general_occupied = status.get('parked_vehicles', 0) - disabled_occupied - ev_occupied
        self.status_labels['available_general'].setText(f'잔여 일반: {6 - general_occupied}대')
        
        info = ""
        for v in vehicles:
            status_text = "🅿️ 주차완료" if v.is_parked else "🚗 이동중"
            spot_text = f" ({v.parked_spot}번)" if v.parked_spot else ""
            type_parts = []
            if v.elec: type_parts.append("⚡전기")
            if v.disabled: type_parts.append("♿장애인")
            type_text = f"[{'+'.join(type_parts)}]" if type_parts else "[일반]"
            info += f"TAG_{v.tag_id}{type_text}: {status_text}{spot_text} ({v.current_position[0]:.0f},{v.current_position[1]:.0f})\n"
        self.vehicles_text.setText(info)

    def show_illegal_parking_popup(self, message):
        """불법 주차 경고 팝업을 표시하는 메서드"""
        msg_box = QMessageBox(self)
        msg_box.setIcon(QMessageBox.Warning)
        msg_box.setWindowTitle("불법 주차 감지")
        msg_box.setText("미등록 차량이 주차하였습니다.")
        msg_box.setInformativeText(message) # 추가 정보 (어떤 차가 어디에)
        msg_box.setStandardButtons(QMessageBox.Ok)
        msg_box.exec_()

    def ros_callback(self, status):
        self.update_signal.emit(status)

    def ros_illegal_parking_callback(self, tag_id, spot_id):
        """ROS 노드로부터 불법 주차 신호를 받아 처리하는 콜백"""
        spot_type = "장애인 주차구역" if spot_id in [1, 6, 7] else "전기차 충전구역"
        message = f"차량 TAG_{tag_id}이(가) {spot_type}({spot_id}번)에 주차했습니다."
        self.illegal_parking_signal.emit(message)

    def start_ros_node(self):
        try:
            rclpy.init()
            # 노드 생성 시 두 개의 콜백 함수를 전달
            self.ros_node = ParkingExeNode(self.ros_callback, self.ros_illegal_parking_callback)
            self.ros_callback(self.ros_node.get_system_status())
            rclpy.spin(self.ros_node)
        except Exception as e:
            print(f"ROS 노드 오류: {e}")

    def closeEvent(self, event):
        if self.ros_node: self.ros_node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        event.accept()

def main(args=None):
    app = QApplication(sys.argv)
    window = ParkingExeMainWindow()
    ros_thread = threading.Thread(target=window.start_ros_node)
    ros_thread.daemon = True
    window.ros_thread = ros_thread
    ros_thread.start()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()