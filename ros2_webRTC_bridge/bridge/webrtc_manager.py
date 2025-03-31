# webrtc_manager.py
import asyncio
import json
from aiortc import RTCPeerConnection, RTCSessionDescription,RTCIceServer,RTCConfiguration


class WebRTCManager:
    """호스트와 클라이언트 역할을 모두 수행할 수 있는 WebRTC 통신 관리 클래스
    다중 데이터 채널을 지원합니다."""
    
    def __init__(self):
        self.pc = None
        self.channels = {}  # 채널 이름을 키로 사용하는 딕셔너리
        self.event_loop = None
        self.pcs = set()
        self.role = None  # 'host' 또는 'client'
        self.on_datachannel_callbacks = {}  # 채널 연결 시 콜백
        self.message_callbacks = {}  # 메시지 수신 시 콜백
        
    async def initialize(self, loop, role='host'):
        """WebRTC 연결을 초기화합니다
        
        Args:
            loop: 비동기 이벤트 루프
            role: 'host' 또는 'client'
        """
        self.event_loop = loop
        self.role = role
        
        # ICE 서버 설정
        ice_servers = []
        ice_servers.append(RTCIceServer(urls=["stun:stun.l.google.com:19302"]))
        ice_servers.append(RTCIceServer(urls=["stun:stun1.l.google.com:19302"]))
        
        config = RTCConfiguration(iceServers=ice_servers)
        
        try:
            self.pc = RTCPeerConnection(config)
            print("RTCPeerConnection created with ICE servers:", [server["urls"] for server in config["iceServers"]])
        except Exception as e:
            print(f"Failed to create RTCPeerConnection with ICE servers: {e}")
            print("Creating RTCPeerConnection without ICE servers. External communication may not work.")
            self.pc = RTCPeerConnection()
        
        # 피어 연결 추가
        self.pcs.add(self.pc)
        
        # ICE 연결 상태 변화 이벤트 핸들러
        @self.pc.on("iceconnectionstatechange")
        def on_iceconnectionstatechange():
            print(f"ICE 연결 상태: {self.pc.iceConnectionState}")
        
        # 데이터 채널 이벤트 핸들러
        @self.pc.on("datachannel")
        def on_datachannel(channel):
            channel_name = channel.label
            print(f"데이터 채널 연결됨: {channel_name} (역할: {self.role})")
            
            # 채널 저장
            self.channels[channel_name] = channel
            
            # 데이터 채널 콜백 실행 (기존 코드)
            if channel_name in self.on_datachannel_callbacks:
                callback = self.on_datachannel_callbacks[channel_name]
                asyncio.run_coroutine_threadsafe(
                    callback(channel),
                    self.event_loop
                )
            
            # 메시지 콜백 설정 (추가 코드)
            if channel_name in self.message_callbacks:
                callback = self.message_callbacks[channel_name]
                
                @channel.on("message")
                def on_message(message):
                    callback(channel, message)
                    
                print(f"{channel_name} 채널에 저장된 메시지 핸들러를 적용했습니다")
    
    def set_datachannel_callback(self, channel_name, callback):
        """특정 데이터 채널이 열렸을 때 호출할 콜백을 설정합니다"""
        self.on_datachannel_callbacks[channel_name] = callback
    
    def set_channel_callback(self, channel_name, callback):
        """특정 채널에서 메시지를 수신했을 때 호출할 콜백을 설정합니다"""
        if channel_name in self.channels:
            channel = self.channels[channel_name]
            
            @channel.on("message")
            def on_message(message):
                # 메시지 수신 시 콜백 호출
                callback(channel, message)
                
            print(f"{channel_name} 채널의 메시지 핸들러가 설정되었습니다")
        else:
            # 향후 채널이 생성/연결될 때 사용할 콜백 저장
            self.message_callbacks[channel_name] = callback
            print(f"{channel_name} 채널의 메시지 콜백이 저장되었습니다 (채널이 열리면 적용됩니다)")
    
    async def create_channel(self, channel_name):
        """호스트 역할일 때 새 데이터 채널을 생성합니다"""
        if self.role != 'host':
            raise ValueError("호스트 역할일 때만 채널을 생성할 수 있습니다")
        
        channel = self.pc.createDataChannel(channel_name)
        self.channels[channel_name] = channel
        print(f"데이터 채널 생성: {channel_name}")
        
        # 채널이 만들어졌을 때 저장된 메시지 콜백이 있으면 적용
        if channel_name in self.message_callbacks:
            callback = self.message_callbacks[channel_name]
            
            @channel.on("message")
            def on_message(message):
                callback(channel, message)
                
            print(f"{channel_name} 채널에 저장된 메시지 핸들러를 적용했습니다")
        
        return channel
    
    def get_channel(self, channel_name):
        """지정된 이름의 데이터 채널을 반환합니다"""
        return self.channels.get(channel_name)
        
    def get_all_channels(self):
        """모든 데이터 채널을 반환합니다"""
        return self.channels
    
    async def create_offer(self):
        """Offer SDP를 생성합니다 (호스트 역할)"""
        if self.role != 'host':
            raise ValueError("호스트 역할일 때만 Offer를 생성할 수 있습니다")
        
        offer = await self.pc.createOffer()
        await self.pc.setLocalDescription(offer)
        offer_json = json.dumps({"sdp": self.pc.localDescription.sdp, "type": self.pc.localDescription.type})
        
        print("=== 생성된 Offer SDP ===")
        print(offer_json)
        print("위 SDP를 다른 컴퓨터에 붙여넣으세요.")
        
        return offer_json
    
    async def create_answer(self, offer_sdp):
        """Answer SDP를 생성합니다 (클라이언트 역할)"""
        if self.role != 'client':
            raise ValueError("클라이언트 역할일 때만 Answer를 생성할 수 있습니다")
        
        offer = RTCSessionDescription(**json.loads(offer_sdp))
        await self.pc.setRemoteDescription(offer)
        
        answer = await self.pc.createAnswer()
        await self.pc.setLocalDescription(answer)
        answer_json = json.dumps({"sdp": self.pc.localDescription.sdp, "type": self.pc.localDescription.type})
        
        print("=== 생성된 Answer SDP ===")
        print(answer_json)
        print("위 SDP를 호스트 컴퓨터에 붙여넣으세요.")
        
        return answer_json
    
    async def handle_answer(self, answer_sdp=None):
        """원격 측의 Answer SDP를 처리합니다 (호스트 역할)"""
        if self.role != 'host':
            raise ValueError("호스트 역할일 때만 Answer를 처리할 수 있습니다")
        
        if answer_sdp is None:
            answer_sdp = input("다른 컴퓨터에서 받은 Answer SDP를 입력하세요: ")
        
        answer = RTCSessionDescription(**json.loads(answer_sdp))
        await self.pc.setRemoteDescription(answer)
    
    async def wait_for_channel_open(self, channel_name):
        """특정 데이터 채널이 열릴 때까지 대기합니다"""
        timeout = 30  # 최대 30초 대기
        print(f"{channel_name} 데이터 채널 대기 중...")
        while channel_name not in self.channels and timeout > 0:
            print(f"{channel_name} 데이터 채널 대기 중...")
            await asyncio.sleep(1)
            timeout -= 1
            
        if channel_name not in self.channels:
            print(f"{channel_name} 데이터 채널 연결 실패: 시간 초과")
            return False
            
        channel = self.channels[channel_name]
        while channel.readyState != "open":
            print(f"{channel_name} 데이터 채널 상태: {channel.readyState}, 대기 중...")
            await asyncio.sleep(1)
        
        print(f"{channel_name} 데이터 채널이 열렸습니다!")
        return True
    
    async def wait_for_all_channels_open(self, channel_names):
        """여러 데이터 채널이 모두 열릴 때까지 대기합니다"""
        results = {}
        for channel_name in channel_names:
            result = await self.wait_for_channel_open(channel_name)
            results[channel_name] = result
        
        return all(results.values())
    
    async def send_data(self, channel_name, data):
        """지정된 WebRTC 데이터 채널을 통해 데이터를 전송합니다"""
        if channel_name in self.channels and self.channels[channel_name].readyState == "open":
            channel = self.channels[channel_name]
            try:
                channel.send(data)
                return True
            except Exception as e:
                print(f"{channel_name} 데이터 전송 실패: {e}")
                return False
        else:
            print(f"{channel_name} 데이터 채널이 열려 있지 않습니다.")
            return False
    
    async def close_connections(self):
        """모든 WebRTC 연결을 종료합니다"""
        # 모든 채널 닫기
        for channel_name, channel in self.channels.items():
            if channel.readyState != "closed":
                channel.close()
                print(f"{channel_name} 데이터 채널을 닫았습니다.")
        
        # 모든 피어 연결 닫기
        for pc in self.pcs:
            await pc.close()
            
        self.channels.clear()
        self.pcs.clear()
        print("모든 WebRTC 연결이 종료되었습니다.")