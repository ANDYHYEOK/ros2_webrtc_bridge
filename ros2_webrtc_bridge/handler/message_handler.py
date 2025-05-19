import json
import base64
import asyncio

class MessageHandler:
    """메시지 처리 및 변환을 담당하는 클래스"""
    
    @staticmethod
    def process_scan(msg):
        """LaserScan 메시지를 JSON으로 변환"""
        scan_data = {
            "type": "laserscan",
            "header": {
                "stamp": {"sec": msg.header.stamp.sec, "nanosec": msg.header.stamp.nanosec},
                "frame_id": msg.header.frame_id
            },
            "angle_min": msg.angle_min,
            "angle_max": msg.angle_max,
            "angle_increment": msg.angle_increment,
            "time_increment": msg.time_increment,
            "scan_time": msg.scan_time,
            "range_min": msg.range_min,
            "range_max": msg.range_max,
            "ranges": list(msg.ranges),
            "intensities": list(msg.intensities)
        }
        return scan_data
    
    @staticmethod
    def process_image(msg):
        """CompressedImage 메시지를 JSON으로 변환"""
        # 이미지 데이터를 base64로 인코딩
        image_data = base64.b64encode(msg.data).decode('utf-8')
        image_msg = {
            "type": "compressed_image",
            "header": {
                "stamp": {"sec": msg.header.stamp.sec, "nanosec": msg.header.stamp.nanosec},
                "frame_id": msg.header.frame_id
            },
            "format": msg.format,
            "data": image_data
        }
        return image_msg
    
    @staticmethod
    def process_twist(msg):
        """Twist 메시지를 JSON으로 변환"""
        twist_data = {
            "type": "twist",
            "linear": {
                "x": msg.linear.x,
                "y": msg.linear.y,
                "z": msg.linear.z
            },
            "angular": {
                "x": msg.angular.x,
                "y": msg.angular.y,
                "z": msg.angular.z
            }
        }
        return twist_data
    
    @staticmethod
    def convert_scan_json_to_msg(json_data):
        """JSON 데이터를 LaserScan 메시지로 변환"""
        from sensor_msgs.msg import LaserScan
        
        msg = LaserScan()
        # 헤더 설정
        msg.header.stamp.sec = json_data["header"]["stamp"]["sec"]
        msg.header.stamp.nanosec = json_data["header"]["stamp"]["nanosec"]
        msg.header.frame_id = json_data["header"]["frame_id"]
        
        # 데이터 설정
        msg.angle_min = json_data["angle_min"]
        msg.angle_max = json_data["angle_max"]
        msg.angle_increment = json_data["angle_increment"]
        msg.time_increment = json_data["time_increment"]
        msg.scan_time = json_data["scan_time"]
        msg.range_min = json_data["range_min"]
        msg.range_max = json_data["range_max"]
        msg.ranges = json_data["ranges"]
        msg.intensities = json_data["intensities"]
        
        return msg
    
    @staticmethod
    def convert_image_json_to_msg(json_data):
        """JSON 데이터를 CompressedImage 메시지로 변환"""
        from sensor_msgs.msg import CompressedImage
        
        msg = CompressedImage()
        # 헤더 설정
        msg.header.stamp.sec = json_data["header"]["stamp"]["sec"]
        msg.header.stamp.nanosec = json_data["header"]["stamp"]["nanosec"]
        msg.header.frame_id = json_data["header"]["frame_id"]
        
        # 데이터 설정
        msg.format = json_data["format"]
        # base64 디코딩
        msg.data = base64.b64decode(json_data["data"])
        
        return msg
    
    @staticmethod
    def convert_twist_json_to_msg(json_data):
        """JSON 데이터를 Twist 메시지로 변환"""
        from geometry_msgs.msg import Twist, Vector3
        
        msg = Twist()
        # 선형 속도 설정
        msg.linear.x = json_data["linear"]["x"]
        msg.linear.y = json_data["linear"]["y"]
        msg.linear.z = json_data["linear"]["z"]
        
        # 각속도 설정
        msg.angular.x = json_data["angular"]["x"]
        msg.angular.y = json_data["angular"]["y"]
        msg.angular.z = json_data["angular"]["z"]
        
        return msg

# 메시지 타입별 처리 및 변환 함수 매핑
MESSAGE_HANDLERS = {
    'LaserScan': {
        'to_json': MessageHandler.process_scan,
        'from_json': MessageHandler.convert_scan_json_to_msg
    },
    'CompressedImage': {
        'to_json': MessageHandler.process_image,
        'from_json': MessageHandler.convert_image_json_to_msg
    },
    'Twist': {
        'to_json': MessageHandler.process_twist,
        'from_json': MessageHandler.convert_twist_json_to_msg
    }
}

def send_msg_to_webrtc(node, webrtc_manager, event_loop, topic_to_channel, topic, msg_type, msg):
    """메시지를 WebRTC를 통해 전송"""
    # 토픽에 맞는 채널 이름 가져오기
    channel_name = topic_to_channel[topic]
    
    # 메시지 타입에 맞는 핸들러 함수 가져와서 처리
    if msg_type in MESSAGE_HANDLERS:
        # JSON 데이터로 변환
        json_data = MESSAGE_HANDLERS[msg_type]['to_json'](msg)
        # 토픽 정보 추가
        json_data["topic"] = topic
        
        # 비동기 전송
        asyncio.run_coroutine_threadsafe(
            webrtc_manager.send_data(channel_name, json.dumps(json_data)),
            event_loop
        )
        node.get_logger().info(f"{msg_type} 메시지를 WebRTC로 전송: {topic}")
    else:
        node.get_logger().error(f"지원되지 않는 메시지 타입: {msg_type}")

def receive_msg_from_webrtc(node, json_str):
    """WebRTC로부터 받은 JSON 문자열을 ROS 메시지로 변환"""
    try:
        json_data = json.loads(json_str)
        msg_type = json_data.get("type")
        topic = json_data.get("topic")
        
        if msg_type == "laserscan":
            # LaserScan 메시지로 변환
            msg = MESSAGE_HANDLERS['LaserScan']['from_json'](json_data)
            node.get_logger().info(f"WebRTC에서 LaserScan 메시지 수신: {topic}")
            return topic, 'LaserScan', msg
            
        elif msg_type == "compressed_image":
            # CompressedImage 메시지로 변환
            msg = MESSAGE_HANDLERS['CompressedImage']['from_json'](json_data)
            node.get_logger().info(f"WebRTC에서 CompressedImage 메시지 수신: {topic}")
            return topic, 'CompressedImage', msg
            
        elif msg_type == "twist":
            # Twist 메시지로 변환
            msg = MESSAGE_HANDLERS['Twist']['from_json'](json_data)
            node.get_logger().info(f"WebRTC에서 Twist 메시지 수신: {topic}")
            return topic, 'Twist', msg
            
        else:
            node.get_logger().error(f"알 수 없는 메시지 타입: {msg_type}")
            return None, None, None
            
    except Exception as e:
        node.get_logger().error(f"메시지 변환 중 오류: {e}")
        return None, None, None