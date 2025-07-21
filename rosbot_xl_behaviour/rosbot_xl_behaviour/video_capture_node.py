import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime

class VideoCaptureNode(Node):
    def __init__(self):
        super().__init__('video_capture_node')

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        self.recording = False
        self.video_writer = None

        self.service = self.create_service(
            SetBool,
            'toggle_recording',
            self.handle_toggle_recording
        )

    def image_callback(self, msg):
        if not self.recording:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.video_writer.write(cv_image)

    def handle_toggle_recording(self, request, response):
        if request.data: 
            if self.recording:
                response.success = False
                response.message = "Ya estaba grabando."
            else:
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                output_dir = os.path.expanduser('/home/paula/security_bot_ws/src/rosbot_xl_behaviour/recordings')
                os.makedirs(output_dir, exist_ok=True)
                timestamp = datetime.now().strftime('%Y-%m-%d_%H:%M:%S')
                filename = f"video_{timestamp}.avi"
                filepath = os.path.join(output_dir, filename)

                self.video_writer = cv2.VideoWriter(filepath, fourcc, 20.0, (320, 240))
                self.recording = True
                self.get_logger().info('Grabación iniciada.')
                response.success = True
                response.message = "Grabación iniciada."
        else:  
            if not self.recording:
                response.success = False
                response.message = "No se estaba grabando."
            else:
                self.recording = False
                self.video_writer.release()
                self.get_logger().info('Grabación detenida.'
                                       )
                response.success = True
                response.message = "Grabación detenida."
        return response

def main(args=None):
    rclpy.init(args=args)
    node = VideoCaptureNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
