#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from uwb_tracking.msg import UWBTag
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

class UWBReceiver(Node):
    def __init__(self):
        super().__init__('uwb_receiver')
        
        # 파라미터 선언
        self.declare_parameter('coordinate_scale', 0.01)  # cm to m
        self.declare_parameter('parking_width', 19.0)
        self.declare_parameter('parking_height', 19.0)
        
        # 파라미터 가져오기
        self.scale = self.get_parameter('coordinate_scale').get_parameter_value().double_value
        
        # 구독자
        self.uwb_sub = self.create_subscription(
            UWBTag, '/uwb/tag_data', 
            self.uwb_callback, 10)
        
        # 발행자들
        self.position_pubs = {}  # tag_id별 위치 발행자
        self.markers_pub = self.create_publisher(MarkerArray, '/uwb/markers', 10)
        
        # 데이터 저장
        self.active_tags = {}
        
        # 차량 타입별 설정
        self.vehicle_configs = {
            0: {'name': '일반차', 'color': [0.7, 0.7, 0.7, 0.8], 'scale': [1.8, 4.2, 1.5]},
            1: {'name': '전기차', 'color': [0.0, 1.0, 0.0, 0.8], 'scale': [1.8, 4.2, 1.5]},
            2: {'name': '장애인차', 'color': [0.0, 0.5, 1.0, 0.8], 'scale': [1.8, 4.2, 1.5]}
        }
        
        # 시각화 타이머
        self.viz_timer = self.create_timer(0.1, self.publish_markers)
        
        self.get_logger().info('UWB Receiver started - listening for DDS messages')
    
    def uwb_callback(self, msg):
        """UWB DDS 메시지 수신"""
        
        # 데이터 검증
        if not self.validate_data(msg):
            return
        
        # 좌표 변환 (0~1900 → 미터)
        x_meters = msg.position_x * self.scale
        y_meters = msg.position_y * self.scale
        
        # 태그별 위치 발행자 생성
        if msg.tag_id not in self.position_pubs:
            topic_name = f'/uwb/tag_{msg.tag_id}/position'
            self.position_pubs[msg.tag_id] = self.create_publisher(
                PointStamped, topic_name, 10)
        
        # PointStamped 메시지 생성
        position_msg = PointStamped()
        position_msg.header.stamp = self.get_clock().now().to_msg()
        position_msg.header.frame_id = 'map'
        position_msg.point.x = x_meters
        position_msg.point.y = y_meters
        position_msg.point.z = 0.0
        
        # 위치 발행
        self.position_pubs[msg.tag_id].publish(position_msg)
        
        # 활성 태그 데이터 저장
        self.active_tags[msg.tag_id] = {
            'x': x_meters,
            'y': y_meters,
            'z': 0.0,
            'vehicle_type': msg.vehicle_type,
            'raw_x': msg.position_x,
            'raw_y': msg.position_y,
            'timestamp': self.get_clock().now()
        }
        
        # 로그 출력
        vehicle_name = self.vehicle_configs[msg.vehicle_type]['name']
        self.get_logger().info(
            f'Tag {msg.tag_id} ({vehicle_name}): '
            f'Raw({msg.position_x}, {msg.position_y}) → '
            f'Map({x_meters:.2f}m, {y_meters:.2f}m)'
        )
    
    def validate_data(self, msg):
        """데이터 검증"""
        if not (1 <= msg.tag_id <= 10):
            self.get_logger().warn(f'Invalid tag_id: {msg.tag_id}')
            return False
        
        if not (0 <= msg.position_x <= 1900):
            self.get_logger().warn(f'Invalid position_x: {msg.position_x}')
            return False
            
        if not (0 <= msg.position_y <= 1900):
            self.get_logger().warn(f'Invalid position_y: {msg.position_y}')
            return False
        
        if not (0 <= msg.vehicle_type <= 2):
            self.get_logger().warn(f'Invalid vehicle_type: {msg.vehicle_type}')
            return False
        
        return True
    
    def publish_markers(self):
        """시각화 마커 발행"""
        if not self.active_tags:
            return
        
        marker_array = MarkerArray()
        
        for tag_id, data in self.active_tags.items():
            config = self.vehicle_configs[data['vehicle_type']]
            
            # 차량 마커
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = 'map'
            marker.ns = 'vehicles'
            marker.id = tag_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # 위치
            marker.pose.position.x = data['x']
            marker.pose.position.y = data['y']
            marker.pose.position.z = config['scale'][2] / 2
            marker.pose.orientation.w = 1.0
            
            # 크기
            marker.scale.x = config['scale'][0]
            marker.scale.y = config['scale'][1]
            marker.scale.z = config['scale'][2]
            
            # 색상
            marker.color.r = config['color'][0]
            marker.color.g = config['color'][1]
            marker.color.b = config['color'][2]
            marker.color.a = config['color'][3]
            
            marker_array.markers.append(marker)
            
            # 라벨
            label = Marker()
            label.header = marker.header
            label.ns = 'labels'
            label.id = tag_id + 100
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            
            label.pose.position.x = data['x']
            label.pose.position.y = data['y']
            label.pose.position.z = config['scale'][2] + 0.5
            label.pose.orientation.w = 1.0
            
            label.scale.z = 0.6
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            
            label.text = f"ID:{tag_id}\n{config['name']}\n({data['raw_x']},{data['raw_y']})"
            
            marker_array.markers.append(label)
        
        self.markers_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = UWBReceiver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
