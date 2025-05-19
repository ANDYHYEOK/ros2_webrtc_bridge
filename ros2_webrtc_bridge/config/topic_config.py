from sensor_msgs.msg import LaserScan, CompressedImage
from geometry_msgs.msg import Twist

# 메시지 타입 매핑
MESSAGE_TYPES = {
    'LaserScan': LaserScan,
    'CompressedImage': CompressedImage,
    'Twist':Twist,
}

# 통합된 토픽 설정 - 콜백 함수는 나중에 동적으로 할당
TOPICS = {
    # 토픽 이름: {
    #    'msg_type': 메시지 타입 문자열,
    #    'channel': 채널 이름,
    #    'qos': QoS 값,
    #    'direction': 통신 방향 ('publish', 'subscribe', 'both')
    # }
    '/cmd_vel': {
        'msg_type': 'Twist',
        'channel': 'twist',
        'qos': 10,
        'direction': 'subscribe'  # 양방향 통신
    },
    '/scan': {
        'msg_type': 'LaserScan',
        'channel': 'laserscan',
        'qos': 10,
        'direction': 'publish'  # 양방향 통신
    },
    '/camera/color/image_raw/compressed': {
        'msg_type': 'CompressedImage',
        'channel': 'image',
        'qos': 10,
        'direction': 'publish'  # 양방향 통신
    },
}

def get_topic_to_channel_mapping():
    """토픽과 채널 이름 매핑을 반환합니다"""
    return {topic: config['channel'] for topic, config in TOPICS.items()}

def get_channel_names():
    """모든 채널 이름 목록을 반환합니다"""
    return [config['channel'] for config in TOPICS.values()]

def get_subscription_topics(role):
    """현재 역할에 맞는 구독 토픽 목록을 반환합니다"""
    return {topic: config for topic, config in TOPICS.items() 
            if config['direction'] in ['both', 'subscribe']}

def get_publication_topics(role):
    """현재 역할에 맞는 발행 토픽 목록을 반환합니다"""
    return {topic: config for topic, config in TOPICS.items() 
            if config['direction'] in ['both', 'publish']}