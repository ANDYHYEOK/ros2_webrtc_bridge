import asyncio
import traceback
import rclpy
from ros2_webRTC_bridge.bridge.ros_webrtc_bridge import ROSWebRTCBridge,run_host

async def main():
    """호스트 모드로 실행합니다"""
    print("1. ROS 초기화 시작")
    # ROS 초기화
    rclpy.init()
    
    print("2. ROS 노드 생성 시작")
    # ROS 노드 생성
    ros_node = ROSWebRTCBridge('ros_webrtc_bridge')
    
    try:
        print("3. 호스트 실행 함수 호출 시작")
        # 호스트 모드로 실행
        await run_host(ros_node)
    except Exception as e:
        print(f"오류 발생: {e}")
        print("상세 오류 정보:")
        traceback.print_exc()
    finally:
        print("4. ROS 종료 작업 시작")
        # ROS 종료
        rclpy.shutdown()
        print("프로그램이 종료되었습니다.")

if __name__ == "__main__":
    print("ROS-WebRTC 브릿지를 호스트 모드로 시작합니다...")
    try:
        asyncio.run(main())
    except Exception as e:
        print(f"메인 루프 오류: {e}")
        traceback.print_exc()