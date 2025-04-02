#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage,Image
import cv2
import numpy as np
from cv_bridge import CvBridge
import time

class VideoCompressedPublisher(Node):
    def __init__(self):
        super().__init__('video_compressed_publisher')

        
        self.subscription = self.create_subscription(Image,'camera/color/image_raw', self.image_callback, 10)

        self.bridge = CvBridge()
        
        # Create a publisher for compressed images
        self.publisher = self.create_publisher(
            CompressedImage, 
            '/camera/color/image_raw/compressed',
            10
        )
        

    def image_callback(self,msg):
        
        img = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        comp_img = self.bridge.cv2_to_compressed_imgmsg(img)
        self.publisher.publish(comp_img)
        
            

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