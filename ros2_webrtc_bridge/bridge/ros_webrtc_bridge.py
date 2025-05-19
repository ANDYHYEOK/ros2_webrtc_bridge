# bridge/ros_webrtc_bridge.py
import rclpy
from rclpy.node import Node
import asyncio
from concurrent.futures import ThreadPoolExecutor

from ros2_webrtc_bridge.config.topic_config import (
    MESSAGE_TYPES, TOPICS, 
    get_topic_to_channel_mapping, get_channel_names,
    get_subscription_topics, get_publication_topics
)
from ros2_webrtc_bridge.handler.message_handler import send_msg_to_webrtc, receive_msg_from_webrtc
from ros2_webrtc_bridge.bridge.webrtc_manager import WebRTCManager

class ROSWebRTCBridge(Node):
    """ROS 토픽을 구독하고 WebRTC를 통해 데이터를 전송하는 노드"""
    
    def __init__(self, node_name='ros_webrtc_bridge'):
        super().__init__(node_name)
        
        # WebRTC 관리자와 이벤트 루프
        self.webrtc_manager = None
        self.event_loop = None
        self.role = None  # 'host' 또는 'client'
        
        # 토픽-채널 매핑
        self.topic_to_channel = get_topic_to_channel_mapping()
        
        # 구독 및 발행 저장 (변수명 변경)
        self.topic_subscriptions = {}  # 'subscriptions' 대신 'topic_subscriptions'
        self.topic_publishers = {}     # 'publishers' 대신 'topic_publishers'
        
        self.get_logger().info(f'{node_name} 초기화됨')
    
    def initialize(self, webrtc_manager, loop, role):
        """WebRTC 관리자와 이벤트 루프를 설정합니다"""
        self.webrtc_manager = webrtc_manager
        self.event_loop = loop
        self.role = role
        self.get_logger().info(f'WebRTC 관리자가 설정되었습니다 (역할: {role})')
    
    def start_subscriptions(self):
        """ROS 토픽 구독을 시작합니다"""
        if not self.webrtc_manager:
            self.get_logger().error('WebRTC 관리자가 설정되지 않았습니다')
            return False
        
        # 구독할 토픽 가져오기
        sub_topics = get_subscription_topics(self.role)
        
        # 토픽 설정에 따라 구독 생성
        for topic, config in sub_topics.items():
            msg_type_str = config['msg_type']
            qos = config['qos']
            
            # 메시지 타입 클래스 가져오기
            if msg_type_str in MESSAGE_TYPES:
                msg_type_class = MESSAGE_TYPES[msg_type_str]
                
                # 콜백 함수 생성
                def create_callback(topic, msg_type):
                    def callback(msg):
                        send_msg_to_webrtc(
                            self, self.webrtc_manager, self.event_loop, 
                            self.topic_to_channel, topic, msg_type, msg
                        )
                    return callback
                
                # 구독 생성
                callback = create_callback(topic, msg_type_str)
                self.topic_subscriptions[topic] = self.create_subscription(
                    msg_type_class, topic, callback, qos)
                
                self.get_logger().info(f'{msg_type_str} 구독 시작: {topic}')
            else:
                self.get_logger().error(f'알 수 없는 메시지 타입: {msg_type_str}')
        
        self.get_logger().info('=== ROS 토픽 구독이 시작되었습니다 ===')
        self.get_logger().info(f'구독 중인 토픽: {list(self.topic_subscriptions.keys())}')
        
        return len(self.topic_subscriptions) > 0
    
    def setup_publishers(self):
        """발행할 토픽에 대한 퍼블리셔를 설정합니다"""
        # 발행할 토픽 가져오기
        pub_topics = get_publication_topics(self.role)
        
        # 토픽 설정에 따라 퍼블리셔 생성
        for topic, config in pub_topics.items():
            msg_type_str = config['msg_type']
            qos = config['qos']
            
            # 메시지 타입 클래스 가져오기
            if msg_type_str in MESSAGE_TYPES:
                msg_type_class = MESSAGE_TYPES[msg_type_str]
                
                # 퍼블리셔 생성
                self.topic_publishers[topic] = self.create_publisher(
                    msg_type_class, topic, qos)
                
                self.get_logger().info(f'{msg_type_str} 퍼블리셔 생성: {topic}')
            else:
                self.get_logger().error(f'알 수 없는 메시지 타입: {msg_type_str}')
        
        self.get_logger().info('=== ROS 토픽 퍼블리셔가 설정되었습니다 ===')
        self.get_logger().info(f'발행 중인 토픽: {list(self.topic_publishers.keys())}')
        
        return len(self.topic_publishers) > 0
    
    def handle_webrtc_message(self, channel, message):
        """WebRTC로부터 받은 메시지를 처리하고 ROS 토픽으로 발행합니다"""
        topic, msg_type, msg = receive_msg_from_webrtc(self, message)
        
        if topic and msg_type and msg:
            # 해당 토픽에 대한 퍼블리셔가 있는지 확인
            if topic in self.topic_publishers:
                # 메시지 발행
                self.topic_publishers[topic].publish(msg)
                self.get_logger().info(f"WebRTC에서 받은 {msg_type} 메시지를 {topic}으로 발행")
            else:
                self.get_logger().warn(f"수신된 토픽 {topic}에 대한 퍼블리셔가 없습니다")
    
    async def setup_host(self):
        """호스트 모드에서 WebRTC 설정 및 ROS 노드 시작을 관리합니다"""
        if not self.webrtc_manager:
            self.get_logger().error('WebRTC 관리자가 설정되지 않았습니다')
            return False
        
        try:
            print("1. 채널 생성 시작")
            # 모든 채널 이름 가져오기
            channel_names = get_channel_names()
            for channel_name in channel_names:
                await self.webrtc_manager.create_channel(channel_name)
                
                # 데이터 채널 콜백 설정
                self.webrtc_manager.set_channel_callback(
                    channel_name, 
                    lambda channel, message: self.handle_webrtc_message(channel, message)
                )
                
                self.get_logger().info(f'채널 생성: {channel_name}')
            
            # 퍼블리셔 설정
            self.setup_publishers()
            
            print("2. Offer 생성 시작")
            # Offer 생성
            await self.webrtc_manager.create_offer()
            
            print("3. Answer 처리 시작")
            # Answer 처리
            await self.webrtc_manager.handle_answer()
            
            print("4. 채널 연결 대기 시작")
            # 모든 채널이 열릴 때까지 대기
            if await self.webrtc_manager.wait_for_all_channels_open(channel_names):
                print("5. 토픽 구독 시작")
                # 채널이 열리면 토픽 구독 시작
                self.start_subscriptions()
                self.get_logger().info('모든 WebRTC 채널이 열렸습니다, ROS 토픽 구독을 시작합니다')
                return True
            else:
                self.get_logger().error('WebRTC 채널 연결 실패')
                return False
        except Exception as e:
            print(f"setup_host 오류: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    async def setup_client(self):
        """클라이언트 모드에서 WebRTC 설정 및 ROS 노드 시작을 관리합니다"""
        if not self.webrtc_manager:
            self.get_logger().error('WebRTC 관리자가 설정되지 않았습니다')
            return False
        
        try:
            print("1. 콜백 설정 시작")
            # 각 채널에 대한 콜백 설정
            channel_names = get_channel_names()
            for channel_name in channel_names:
                # 데이터 채널 콜백 설정
                self.webrtc_manager.set_channel_callback(
                    channel_name, 
                    lambda channel, message: self.handle_webrtc_message(channel, message)
                )
                
                self.get_logger().info(f'채널 콜백 설정: {channel_name}')
            
            # 퍼블리셔 설정
            self.setup_publishers()
            
            print("2. Offer 입력 시작")
            # Offer 입력 받기
            offer_sdp = input("호스트 컴퓨터에서 받은 Offer SDP를 입력하세요: ")
            
            print("3. Answer 생성 시작")
            # Answer 생성 및 출력
            await self.webrtc_manager.create_answer(offer_sdp)
            
            print("4. 채널 연결 대기 시작")
            # 모든 채널이 열릴 때까지 대기
            if await self.webrtc_manager.wait_for_all_channels_open(channel_names):
                print("5. 토픽 구독 시작")
                # 채널이 열리면 토픽 구독 시작
                self.start_subscriptions()
                self.get_logger().info('모든 WebRTC 채널이 열렸습니다, ROS 토픽 구독을 시작합니다')
                return True
            else:
                self.get_logger().error('WebRTC 채널 연결 실패')
                return False
        except Exception as e:
            print(f"setup_client 오류: {e}")
            import traceback
            traceback.print_exc()
            return False
async def run_host(node):
    """호스트 모드로 실행"""
    print("run_host 함수 시작")
    # 비동기 이벤트 루프 가져오기
    loop = asyncio.get_running_loop()
    print("이벤트 루프 가져옴")
    
    try:
        # WebRTC 관리자 생성 및 호스트 모드로 초기화
        webrtc_manager = WebRTCManager()
        print("WebRTCManager 인스턴스 생성 성공")
        
        print("WebRTC 관리자 초기화 시작")
        await webrtc_manager.initialize(loop, role='host')
        print("WebRTC 관리자 초기화 완료")
        
        # ROS 노드 설정
        print("ROS 노드 설정 시작")
        node.initialize(webrtc_manager, loop, 'host')  # 역할 정보 추가
        print("ROS 노드 설정 완료")
        
        # 스레드 풀 생성
        print("스레드 풀 생성")
        executor = ThreadPoolExecutor()
        
        # WebRTC 설정 및 ROS 구독 시작
        print("node.setup_host 호출 시작")
        setup_task = asyncio.create_task(node.setup_host())
        print("node.setup_host 태스크 생성 완료")
        
        # ROS 스핀 실행 (별도 스레드에서)
        print("ROS 스핀 시작")
        _ = loop.run_in_executor(executor, rclpy.spin, node)
        print("ROS 스핀 태스크 생성 완료")
        
        # 설정 완료 대기
        print("setup_task 완료 대기 시작")
        result = await setup_task
        print(f"setup_task 완료: {result}")
        
        if not result:
            print("WebRTC 설정 실패")
            return False
        
        # 사용자가 종료할 때까지 대기 (Ctrl+C 등)
        print("호스트 모드로 실행 중입니다. 종료하려면 Ctrl+C를 누르세요.")
        
        # 종료 이벤트 사용
        shutdown_event = asyncio.Event()
        try:
            print("shutdown_event.wait() 시작")
            await shutdown_event.wait()  # 이벤트가 설정될 때까지 대기 (실질적으로 무한 대기)
            print("shutdown_event.wait() 완료")
        except asyncio.CancelledError:
            print("종료 요청을 받았습니다.")
            
    except Exception as e:
        print(f"run_host 함수 오류: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 정리 작업
        print("run_host 함수 정리 작업 시작")
        if 'setup_task' in locals() and not setup_task.done():
            setup_task.cancel()
        
        if 'executor' in locals():
            executor.shutdown(wait=False)
        
        if 'webrtc_manager' in locals():
            await webrtc_manager.close_connections()
        
        print("run_host 함수 정리 작업 완료")
        return True

async def run_client(node):
    """클라이언트 모드로 실행"""
    print("run_client 함수 시작")
    # 비동기 이벤트 루프 가져오기
    loop = asyncio.get_running_loop()
    print("이벤트 루프 가져옴")
    
    try:
        # WebRTC 관리자 생성 및 클라이언트 모드로 초기화
        webrtc_manager = WebRTCManager()
        print("WebRTCManager 인스턴스 생성 성공")
        
        print("WebRTC 관리자 초기화 시작")
        await webrtc_manager.initialize(loop, role='client')
        print("WebRTC 관리자 초기화 완료")
        
        # ROS 노드 설정
        print("ROS 노드 설정 시작")
        node.initialize(webrtc_manager, loop, 'client')  # 역할 정보 추가
        print("ROS 노드 설정 완료")
        
        # 스레드 풀 생성
        print("스레드 풀 생성")
        executor = ThreadPoolExecutor()
        
        # WebRTC 설정 및 ROS 구독 시작
        print("node.setup_client 호출 시작")
        setup_task = asyncio.create_task(node.setup_client())
        print("node.setup_client 태스크 생성 완료")
        
        # ROS 스핀 실행 (별도 스레드에서)
        print("ROS 스핀 시작")
        _ = loop.run_in_executor(executor, rclpy.spin, node)
        print("ROS 스핀 태스크 생성 완료")
        
        # 설정 완료 대기
        print("setup_task 완료 대기 시작")
        result = await setup_task
        print(f"setup_task 완료: {result}")
        
        if not result:
            print("WebRTC 설정 실패")
            return False
        
        # 사용자가 종료할 때까지 대기 (Ctrl+C 등)
        print("클라이언트 모드로 실행 중입니다. 종료하려면 Ctrl+C를 누르세요.")
        
        # 종료 이벤트 사용
        shutdown_event = asyncio.Event()
        try:
            print("shutdown_event.wait() 시작")
            await shutdown_event.wait()  # 이벤트가 설정될 때까지 대기 (실질적으로 무한 대기)
            print("shutdown_event.wait() 완료")
        except asyncio.CancelledError:
            print("종료 요청을 받았습니다.")
            
    except Exception as e:
        print(f"run_client 함수 오류: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 정리 작업
        print("run_client 함수 정리 작업 시작")
        if 'setup_task' in locals() and not setup_task.done():
            setup_task.cancel()
        
        if 'executor' in locals():
            executor.shutdown(wait=False)
        
        if 'webrtc_manager' in locals():
            await webrtc_manager.close_connections()
        
        print("run_client 함수 정리 작업 완료")
        return True