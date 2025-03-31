#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from cv_bridge import CvBridge
import time

class VideoCompressedPublisher(Node):
    def __init__(self):
        super().__init__('video_compressed_publisher')
        
        # Parameters
        self.declare_parameter('device_id', 0)
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('compression_quality', 80)
        self.declare_parameter('resolution_width', 640)
        self.declare_parameter('resolution_height', 480)
        
        # Get parameters
        self.device_id = self.get_parameter('device_id').value
        self.frame_rate = self.get_parameter('frame_rate').value
        self.compression_quality = self.get_parameter('compression_quality').value
        self.width = self.get_parameter('resolution_width').value
        self.height = self.get_parameter('resolution_height').value
        
        # Create a publisher for compressed images
        self.publisher = self.create_publisher(
            CompressedImage, 
            '/video/compressed',
            10
        )
        
        # Initialize OpenCV video capture
        self.cap = cv2.VideoCapture(self.device_id)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open video device {self.device_id}')
            return
            
        # Set resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Set up timer for publishing at desired frame rate
        period = 1.0 / self.frame_rate
        self.timer = self.create_timer(period, self.timer_callback)
        
        self.get_logger().info(
            f'Video compressed publisher started with:\n'
            f'  - Device ID: {self.device_id}\n'
            f'  - Resolution: {self.width}x{self.height}\n'
            f'  - Frame rate: {self.frame_rate} fps\n'
            f'  - Compression quality: {self.compression_quality}'
        )
        
    def timer_callback(self):
        """Timer callback to publish compressed video frames"""
        try:
            # Capture frame from video device
            ret, frame = self.cap.read()
            
            if not ret:
                self.get_logger().warn('Failed to capture frame')
                return
                
            # Create CompressedImage message
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = 'jpeg'
            
            # Compress the image
            encode_param = [cv2.IMWRITE_JPEG_QUALITY, self.compression_quality]
            result, compressed = cv2.imencode('.jpg', frame, encode_param)
            
            if not result:
                self.get_logger().warn('Failed to compress image')
                return
                
            # Convert compressed image to bytes and add to message
            msg.data = np.array(compressed).tobytes()
            
            # Publish the message
            self.publisher.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {str(e)}')
            
    def __del__(self):
        """Destructor to release the video capture device"""
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
            

def main(args=None):
    rclpy.init(args=args)
    
    node = VideoCompressedPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {str(e)}')
    finally:
        # Cleanup
        if hasattr(node, 'cap') and node.cap.isOpened():
            node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()