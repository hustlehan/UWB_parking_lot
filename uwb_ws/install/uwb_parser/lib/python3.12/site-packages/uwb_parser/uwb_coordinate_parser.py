#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
import json
import re
import time


class UWBControlSystem(Node):
    def __init__(self):
        super().__init__('uwb_control_system')
        
        # 파라미터 선언
        self.declare_parameter('parking_input_topic', '/parking/auth_req')   #서윤 블루투스 결과값 수신
        self.declare_parameter('parking_output_topic', '/parking/barrier_cmd') #주차 진입 차단기에 보내는 명령
        self.declare_parameter('uwb_pos_topic', '/uwb/pos') #재윤 uwb eps32 모듈로부터 수신
        self.declare_parameter('uwb_comp_topic', '/uwb/comp') #수신값 정제해서 publish 하는 값
        self.declare_parameter('track_start_topic', '/uwb/track_start') #uwb 추적 시작 명령 토픽
        self.declare_parameter('track_stop_topic', '/uwb/track_stop') #uwb 추적 중단 명령 토픽
        self.declare_parameter('frame_id', 'uwb_frame') #uwb에 부여하는 추적 번호 id 값
        
        # 파라미터 가져오기
        parking_input_topic = self.get_parameter('parking_input_topic').value
        parking_output_topic = self.get_parameter('parking_output_topic').value
        uwb_pos_topic = self.get_parameter('uwb_pos_topic').value
        uwb_comp_topic = self.get_parameter('uwb_comp_topic').value
        track_start_topic = self.get_parameter('track_start_topic').value
        track_stop_topic = self.get_parameter('track_stop_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # === 차량 번호 → tag_id 매핑 ===
        self.vehicle_to_tag = {}  # 차량번호: tag_id
        self.active_trackings = {}  # tag_id: {"vehicle_id": str, "start_time": float}
        
        # === 출차 처리 관련 변수 ===
        self.pending_exit_tags = {}  # tag_id: {"stage": str, "timestamp": float}
        
        # === Subscribers ===
        # 주차 차단기로부터 차량 ID 수신
        self.parking_subscription = self.create_subscription(
            String,
            parking_input_topic,
            self.parking_callback,
            10
        )
        
        # UWB 모듈로부터 좌표 수신
        self.uwb_subscription = self.create_subscription(
            PointStamped,
            uwb_pos_topic,
            self.uwb_pos_callback,
            10
        )
        
        # 출차 요청 수신 (새로 추가)
        self.exit_subscription = self.create_subscription(
            String,
            '/parking/exit_req',
            self.exit_request_callback,
            10
        )
        
        # 차단기 이벤트 수신 (새로 추가)
        self.barrier_event_subscription = self.create_subscription(
            String,
            '/parking/barrier_event',
            self.barrier_event_callback,
            10
        )
        
        # === Publishers ===
        # 주차 차단기 제어 명령
        self.barrier_cmd_publisher = self.create_publisher(
            String,
            parking_output_topic,
            10
        )
        
        # UWB 추적 시작 명령
        self.track_start_publisher = self.create_publisher(
            String,
            track_start_topic,
            10
        )
        
        # UWB 추적 종료 명령
        self.track_stop_publisher = self.create_publisher(
            String,
            track_stop_topic,
            10
        )
        
        # 처리된 UWB 좌표 발행
        self.uwb_comp_publisher = self.create_publisher(
            PointStamped,
            uwb_comp_topic,
            10
        )
        
        # 차량 타입 정보 발행 (주차장 모니터링용)
        self.vehicle_info_publisher = self.create_publisher(
            String,
            '/uwb/vehicle_info',
            10
        )
        
        # === 통계 변수 ===
        self.total_parking_requests = 0
        self.processed_vehicles = 0
        self.active_tracking_count = 0
        self.total_uwb_messages = 0
        self.processed_uwb_messages = 0
        
        self.get_logger().info(f'UWB Control System initialized')
        self.get_logger().info(f'Listening for auth requests on: {parking_input_topic}')
        self.get_logger().info(f'Listening for exit requests on: /parking/exit_req')
        self.get_logger().info(f'Listening for barrier events on: /parking/barrier_event')
        self.get_logger().info(f'Publishing barrier commands to: {parking_output_topic}')
        self.get_logger().info(f'Listening for UWB positions on: {uwb_pos_topic}')
        self.get_logger().info(f'Publishing UWB commands to: {track_start_topic}, {track_stop_topic}')
        self.get_logger().info(f'Publishing processed coordinates to: {uwb_comp_topic}')

    def parking_callback(self, msg):
        """주차 차단기로부터 차량 ID 수신 처리"""
        self.total_parking_requests += 1
        
        try:
            data = msg.data.strip()
            
            # JSON 형태 처리 (새로운 형식)
            if data.startswith('{') and data.endswith('}'):
                try:
                    vehicle_data = json.loads(data)
                    vehicle_id = vehicle_data.get("vehicle_id")
                    elec = bool(vehicle_data.get("elec", False))  # Boolean으로 직접 처리
                    disabled = bool(vehicle_data.get("disabled", False))  # Boolean으로 직접 처리
                    preferred = vehicle_data.get("preferred", "normal")  # preferred 값 추가
                    gui_mac = vehicle_data.get("gui_mac", "")
                    tag_id = vehicle_data.get("tag_id")  # UWB tag_id 수신 (2자리 정수)
                    destination = vehicle_data.get("destination")  # 목적지 정보 (0, 1, 2)
                    
                    self.get_logger().info(f'Received vehicle JSON: {vehicle_id}, tag_id={tag_id}, elec={elec}, disabled={disabled}, preferred={preferred}, destination={destination}')
                    
                    if vehicle_id and tag_id is not None:
                        # tag_id 유효성 검사 (2자리 정수: 10-99)
                        try:
                            tag_id = int(tag_id)
                            if not (10 <= tag_id <= 99):
                                self.get_logger().error(f'Invalid tag_id: {tag_id}. Must be 2-digit integer (10-99)')
                                return
                        except (ValueError, TypeError):
                            self.get_logger().error(f'Invalid tag_id format: {tag_id}. Must be integer')
                            return
                        
                        # 차량 정보 수신 즉시 차단기 열기 명령 발행
                        barrier_msg = String()
                        barrier_msg.data = '{"gate":"entry", "action":"open"}'
                        self.barrier_cmd_publisher.publish(barrier_msg)
                        
                        # UWB 추적 시작 (수신한 tag_id 사용)
                        self.start_vehicle_tracking(vehicle_id, tag_id, elec, disabled, preferred, destination)
                        self.processed_vehicles += 1
                        
                        vehicle_type = self.get_vehicle_type_description(elec, disabled)
                        self.get_logger().info(f'Vehicle processed: {vehicle_id} (tag_{tag_id:02d}, {vehicle_type}, preferred={preferred}) - Barrier opened')
                    else:
                        if not vehicle_id:
                            self.get_logger().warn('No vehicle_id found in received data')
                        if tag_id is None:
                            self.get_logger().warn('No tag_id found in received data')
                        
                except json.JSONDecodeError:
                    self.get_logger().error(f'Failed to parse JSON: {data}')
                    
            # 기존 "ID=123가5678" 형태 처리 (하위 호환성)
            elif data.startswith("ID="):
                vehicle_id = data[3:]  # "ID=" 제거
                
                self.get_logger().info(f'Received vehicle ID (legacy): {vehicle_id}')
                
                if vehicle_id:
                    # 차량 정보 수신 즉시 차단기 열기 명령 발행
                    barrier_msg = String()
                    barrier_msg.data = "entry_open"
                    self.barrier_cmd_publisher.publish(barrier_msg)
                    
                    # UWB 추적 시작 (기본값 사용 - legacy 지원을 위해 임시 tag_id 할당)
                    temp_tag_id = (hash(vehicle_id) % 90) + 10  # 2자리 정수 범위 (10-99) 보장
                    self.start_vehicle_tracking(vehicle_id, temp_tag_id)
                    self.processed_vehicles += 1
                    
                    self.get_logger().info(f'Vehicle processed: {vehicle_id} (tag_{temp_tag_id:02d}) - Barrier opened')
                    self.get_logger().warn('Legacy format used - consider upgrading to JSON format with tag_id')
                else:
                    self.get_logger().warn('Empty vehicle_id in legacy format')
            else:
                self.get_logger().warn(f'Invalid message format: {data}')
                
        except Exception as e:
            self.get_logger().error(f'Failed to process parking request: {str(e)}')

    def exit_request_callback(self, msg):
        """출차 요청 처리 (1단계: /parking/exit_req 수신)"""
        try:
            data = msg.data.strip()
            
            # tag_id 추출 (JSON 또는 단순 숫자 형태 모두 처리)
            tag_id = None
            if data.startswith('{') and data.endswith('}'):
                # JSON 형태
                try:
                    exit_data = json.loads(data)
                    tag_id = exit_data.get("tag_id")
                except json.JSONDecodeError:
                    self.get_logger().error(f'Failed to parse exit request JSON: {data}')
                    return
            else:
                # 단순 숫자 형태
                try:
                    tag_id = int(data)
                except ValueError:
                    self.get_logger().error(f'Invalid tag_id format in exit request: {data}')
                    return
            
            if tag_id is None:
                self.get_logger().warn('No tag_id found in exit request')
                return
            
            # tag_id 유효성 검사
            if not (10 <= tag_id <= 99):
                self.get_logger().error(f'Invalid tag_id in exit request: {tag_id}. Must be 2-digit integer (10-99)')
                return
            
            # 활성 추적 중인 tag인지 확인
            if tag_id not in self.active_trackings:
                self.get_logger().warn(f'Exit request for inactive tag_{tag_id:02d}')
                return
            
            vehicle_id = self.active_trackings[tag_id]["vehicle_id"]
            self.get_logger().info(f'Exit request received: tag_{tag_id:02d} (Vehicle: {vehicle_id})')
            
            # 출차 처리 상태 추가
            self.pending_exit_tags[tag_id] = {
                "stage": "exit_requested",
                "timestamp": time.time()
            }
            
            # 2단계: 출차 차단기 열기 명령 발행
            barrier_msg = String()
            barrier_msg.data = '{"gate":"exit", "action":"open"}'
            self.barrier_cmd_publisher.publish(barrier_msg)
            
            self.get_logger().info(f'Exit barrier open command sent for tag_{tag_id:02d}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to process exit request: {str(e)}')

    def barrier_event_callback(self, msg):
        """차단기 이벤트 처리 (3단계: /parking/barrier_event 수신)"""
        try:
            data = msg.data.strip()
            
            if data.startswith('{') and data.endswith('}'):
                try:
                    event_data = json.loads(data)
                    gate = event_data.get("gate")
                    state = event_data.get("state")
                    
                    self.get_logger().info(f'Barrier event: gate={gate}, state={state}')
                    
                    # 출차 차단기 닫힘 이벤트 처리
                    if gate == "exit" and state == "closed":
                        self.get_logger().info('Exit barrier closed - processing pending exit tags')
                        
                        # 4단계: 출차 대기 중인 모든 tag 추적 해제
                        for tag_id in list(self.pending_exit_tags.keys()):
                            if self.pending_exit_tags[tag_id]["stage"] == "exit_requested":
                                self.finalize_exit_tracking(tag_id)
                        
                except json.JSONDecodeError:
                    self.get_logger().error(f'Failed to parse barrier event JSON: {data}')
            else:
                self.get_logger().warn(f'Invalid barrier event format: {data}')
                
        except Exception as e:
            self.get_logger().error(f'Failed to process barrier event: {str(e)}')

    def finalize_exit_tracking(self, tag_id):
        """출차 완료 처리 (4단계: UWB 추적 해제)"""
        try:
            if tag_id not in self.active_trackings:
                self.get_logger().warn(f'Cannot finalize exit: tag_{tag_id:02d} not in active trackings')
                return
            
            vehicle_id = self.active_trackings[tag_id]["vehicle_id"]
            
            # UWB 추적 종료 명령 전송
            track_stop_msg = String()
            track_stop_msg.data = f"{vehicle_id},{tag_id}"
            self.track_stop_publisher.publish(track_stop_msg)
            
            # 차량 종료 정보 발행 (주차장 모니터링용)
            vehicle_info_msg = String()
            vehicle_info = {
                "tag_id": tag_id,
                "vehicle_id": vehicle_id,
                "action": "stop_tracking"
            }
            vehicle_info_msg.data = json.dumps(vehicle_info)
            self.vehicle_info_publisher.publish(vehicle_info_msg)
            
            # 추적 정보 정리
            if vehicle_id in self.vehicle_to_tag:
                del self.vehicle_to_tag[vehicle_id]
            del self.active_trackings[tag_id]
            
            # 출차 대기 목록에서 제거
            if tag_id in self.pending_exit_tags:
                del self.pending_exit_tags[tag_id]
            
            self.active_tracking_count -= 1
            
            self.get_logger().info(f'Exit completed: Vehicle {vehicle_id} (tag_{tag_id:02d}) tracking stopped')
            
        except Exception as e:
            self.get_logger().error(f'Failed to finalize exit for tag_{tag_id:02d}: {str(e)}')

    def start_vehicle_tracking(self, vehicle_id, tag_id, elec=False, disabled=False, preferred="normal", destination=None):
        """차량 추적 시작"""
        try:
            # 이미 추적 중인 차량인지 확인
            if vehicle_id in self.vehicle_to_tag:
                existing_tag_id = self.vehicle_to_tag[vehicle_id]
                self.get_logger().info(f'Vehicle {vehicle_id} already being tracked with tag_{existing_tag_id:02d}')
                return
            
            # 같은 tag_id가 이미 사용 중인지 확인
            if tag_id in self.active_trackings:
                existing_vehicle = self.active_trackings[tag_id]["vehicle_id"]
                self.get_logger().warn(f'Tag_{tag_id:02d} already in use by vehicle {existing_vehicle}. Stopping previous tracking.')
                self.stop_vehicle_tracking_by_tag(tag_id)
            
            # 수신한 tag_id 사용
            self.vehicle_to_tag[vehicle_id] = tag_id
            
            # 활성 추적 목록에 추가 (차량 타입 및 목적지 정보 포함)
            self.active_trackings[tag_id] = {
                "vehicle_id": vehicle_id,
                "start_time": time.time(),
                "elec": elec,
                "disabled": disabled,
                "preferred": preferred,
                "destination": destination
            }
            
            # UWB 모듈에 추적 시작 명령 전송
            # 메시지 형식: "vehicle_id,tag_id"
            track_start_msg = String()
            track_start_msg.data = f"{vehicle_id},{tag_id}"
            self.track_start_publisher.publish(track_start_msg)
            
            # 차량 타입 정보 발행 (주차장 모니터링용) - destination 정보 포함
            vehicle_info_msg = String()
            vehicle_info = {
                "tag_id": tag_id,
                "vehicle_id": vehicle_id,
                "elec": elec,
                "disabled": disabled,
                "preferred": preferred,
                "destination": destination,
                "action": "start_tracking"
            }
            vehicle_info_msg.data = json.dumps(vehicle_info)
            self.vehicle_info_publisher.publish(vehicle_info_msg)
            
            self.active_tracking_count += 1
            
            vehicle_type = self.get_vehicle_type_description(elec, disabled)
            destination_desc = self.get_destination_description(destination)
            self.get_logger().info(f'Started tracking: Vehicle {vehicle_id} ({vehicle_type}, preferred={preferred}, destination={destination_desc}) using tag_{tag_id:02d}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to start tracking for {vehicle_id}: {str(e)}')

    def stop_vehicle_tracking_by_tag(self, tag_id):
        """tag_id로 차량 추적 종료"""
        try:
            if tag_id not in self.active_trackings:
                self.get_logger().warn(f'Tag_{tag_id:02d} is not being tracked')
                return
            
            vehicle_id = self.active_trackings[tag_id]["vehicle_id"]
            
            # UWB 모듈에 추적 종료 명령 전송
            track_stop_msg = String()
            track_stop_msg.data = f"{vehicle_id},{tag_id}"
            self.track_stop_publisher.publish(track_stop_msg)
            
            # 차량 종료 정보 발행 (주차장 모니터링용)
            vehicle_info_msg = String()
            vehicle_info = {
                "tag_id": tag_id,
                "vehicle_id": vehicle_id,
                "action": "stop_tracking"
            }
            vehicle_info_msg.data = json.dumps(vehicle_info)
            self.vehicle_info_publisher.publish(vehicle_info_msg)
            
            # 추적 정보 정리
            if vehicle_id in self.vehicle_to_tag:
                del self.vehicle_to_tag[vehicle_id]
            del self.active_trackings[tag_id]
            
            self.active_tracking_count -= 1
            
            self.get_logger().info(f'Stopped tracking: Vehicle {vehicle_id} (tag_{tag_id:02d})')
            
        except Exception as e:
            self.get_logger().error(f'Failed to stop tracking for tag_{tag_id:02d}: {str(e)}')

    def stop_vehicle_tracking(self, vehicle_id):
        """차량 추적 종료"""
        try:
            if vehicle_id not in self.vehicle_to_tag:
                self.get_logger().warn(f'Vehicle {vehicle_id} is not being tracked')
                return
            
            tag_id = self.vehicle_to_tag[vehicle_id]
            
            # UWB 모듈에 추적 종료 명령 전송
            track_stop_msg = String()
            track_stop_msg.data = f"{vehicle_id},{tag_id}"
            self.track_stop_publisher.publish(track_stop_msg)
            
            # 차량 종료 정보 발행 (주차장 모니터링용)
            vehicle_info_msg = String()
            vehicle_info = {
                "tag_id": tag_id,
                "vehicle_id": vehicle_id,
                "action": "stop_tracking"
            }
            vehicle_info_msg.data = json.dumps(vehicle_info)
            self.vehicle_info_publisher.publish(vehicle_info_msg)
            
            # 추적 정보 정리
            del self.vehicle_to_tag[vehicle_id]
            if tag_id in self.active_trackings:
                del self.active_trackings[tag_id]
            
            self.active_tracking_count -= 1
            
            self.get_logger().info(f'Stopped tracking: Vehicle {vehicle_id} (tag_{tag_id:02d})')
            
        except Exception as e:
            self.get_logger().error(f'Failed to stop tracking for {vehicle_id}: {str(e)}')

    def uwb_pos_callback(self, msg):
        """UWB 좌표 데이터 처리 - 미터를 mm로 변환하여 발행"""
        self.total_uwb_messages += 1
        
        try:
            # frame_id에서 tag_id 추출 (예: "tag_10" → 10)
            frame_id = msg.header.frame_id
            if frame_id.startswith("tag_"):
                tag_id = int(frame_id[4:])
                
                # 활성 추적 목록에 있는지 확인
                if tag_id in self.active_trackings:
                    # 핵심 수정: 처리된 좌표 발행 (미터를 mm로 변환)
                    output_msg = PointStamped()
                    output_msg.header.stamp = self.get_clock().now().to_msg()
                    output_msg.header.frame_id = frame_id  # 원본 frame_id 유지
                    output_msg.point.x = msg.point.x * 1000  # 미터 → mm 변환
                    output_msg.point.y = msg.point.y * 1000  # 미터 → mm 변환
                    output_msg.point.z = 0.0
                    
                    self.uwb_comp_publisher.publish(output_msg)
                    self.processed_uwb_messages += 1
                    
                    vehicle_id = self.active_trackings[tag_id]["vehicle_id"]
                    elec = self.active_trackings[tag_id].get("elec", False)
                    disabled = self.active_trackings[tag_id].get("disabled", False)
                    preferred = self.active_trackings[tag_id].get("preferred", "normal")
                    vehicle_type = self.get_vehicle_type_description(elec, disabled)
                    
                    self.get_logger().debug(f'Processed UWB data: Vehicle {vehicle_id} ({vehicle_type}, preferred={preferred}) (tag_{tag_id:02d}) '
                                          f'raw=({msg.point.x:.3f}, {msg.point.y:.3f}) → converted=({output_msg.point.x:.0f}, {output_msg.point.y:.0f})')
                else:
                    self.get_logger().debug(f'Received data for inactive tag_{tag_id:02d}')
            else:
                self.get_logger().warn(f'Invalid frame_id format: {frame_id}')
                
        except Exception as e:
            self.get_logger().warn(f'Failed to process UWB data: {str(e)}')

    def get_destination_description(self, destination):
        """목적지 설명 반환"""
        if destination == 0:
            return "백화점 본관"
        elif destination == 1:
            return "영화관"
        elif destination == 2:
            return "문화시설"
        else:
            return "미지정"

    def get_vehicle_type_description(self, elec, disabled):
        """차량 타입 설명 반환"""
        if elec and disabled:
            return "전기차+장애인"
        elif elec:
            return "전기차"
        elif disabled:
            return "장애인"
        else:
            return "일반차"

    def manual_stop_tracking(self, vehicle_id):
        """수동으로 차량 추적 종료"""
        self.stop_vehicle_tracking(vehicle_id)

    def get_statistics(self):
        """시스템 통계 정보 반환"""
        uwb_success_rate = (self.processed_uwb_messages / self.total_uwb_messages * 100) if self.total_uwb_messages > 0 else 0
        processing_rate = (self.processed_vehicles / self.total_parking_requests * 100) if self.total_parking_requests > 0 else 0
        
        return {
            'total_parking_requests': self.total_parking_requests,
            'processed_vehicles': self.processed_vehicles,
            'processing_rate': processing_rate,
            'active_trackings': self.active_tracking_count,
            'total_uwb_messages': self.total_uwb_messages,
            'processed_uwb_messages': self.processed_uwb_messages,
            'uwb_success_rate': uwb_success_rate
        }

    def list_active_trackings(self):
        """현재 추적 중인 차량 목록 출력"""
        if self.active_trackings:
            self.get_logger().info("=== Active Trackings ===")
            for tag_id, info in self.active_trackings.items():
                duration = time.time() - info["start_time"]
                preferred = info.get("preferred", "normal")
                destination = info.get("destination", None)
                elec = info.get("elec", False)
                disabled = info.get("disabled", False)
                vehicle_type = self.get_vehicle_type_description(elec, disabled)
                destination_desc = self.get_destination_description(destination)
                self.get_logger().info(f"  tag_{tag_id:02d} | {info['vehicle_id']} | {vehicle_type} | preferred={preferred} | destination={destination_desc} | {duration:.1f}s")
        else:
            self.get_logger().info("No active trackings")


def main(args=None):
    rclpy.init(args=args)
    
    uwb_system = UWBControlSystem()
    
    try:
        rclpy.spin(uwb_system)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 통계 정보 출력
        stats = uwb_system.get_statistics()
        uwb_system.get_logger().info(f"=== System Statistics ===")
        uwb_system.get_logger().info(f"  Parking requests: {stats['total_parking_requests']}")
        uwb_system.get_logger().info(f"  Processed vehicles: {stats['processed_vehicles']}")
        uwb_system.get_logger().info(f"  Processing rate: {stats['processing_rate']:.1f}%")
        uwb_system.get_logger().info(f"  Active trackings: {stats['active_trackings']}")
        uwb_system.get_logger().info(f"  UWB messages: {stats['total_uwb_messages']}")
        uwb_system.get_logger().info(f"  Processed UWB: {stats['processed_uwb_messages']}")
        uwb_system.get_logger().info(f"  UWB success rate: {stats['uwb_success_rate']:.1f}%")
        
        uwb_system.list_active_trackings()
        
        uwb_system.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()