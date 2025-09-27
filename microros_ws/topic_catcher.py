#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime

class TopicCatcher(Node):
    def __init__(self):
        super().__init__('topic_catcher')
        
        # 구독자들 생성
        self.auth_sub = self.create_subscription(String, '/parking/auth_req', self.auth_callback, 10)
        self.barrier_sub = self.create_subscription(String, '/parking/barrier_event', self.barrier_callback, 10)
        self.assign_sub = self.create_subscription(String, '/parking/assign_tag', self.assign_callback, 10)
        self.cmd_sub = self.create_subscription(String, '/parking/barrier_cmd', self.cmd_callback, 10)
        self.result_sub = self.create_subscription(String, '/parking/auth_result', self.result_callback, 10)
        
        print("🎯 토픽 캐처 시작! 모든 메시지를 자동으로 캐치합니다...")
    
    def auth_callback(self, msg):
        print(f"\n🔐 [{datetime.now()}] AUTH_REQ: {msg.data}")
    
    def barrier_callback(self, msg):
        print(f"\n🚧 [{datetime.now()}] BARRIER_EVENT: {msg.data}")
    
    def assign_callback(self, msg):
        print(f"\n🏷️  [{datetime.now()}] ASSIGN_TAG: {msg.data}")
    
    def cmd_callback(self, msg):
        print(f"\n📡 [{datetime.now()}] BARRIER_CMD: {msg.data}")
    
    def result_callback(self, msg):
        print(f"\n✅ [{datetime.now()}] AUTH_RESULT: {msg.data}")

def main():
    rclpy.init()
    catcher = TopicCatcher()
    try:
        rclpy.spin(catcher)
    except KeyboardInterrupt:
        print("\n토픽 캐처 종료")
    finally:
        catcher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
