import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
from custom_msgs.msg import Yolo8Inference
import math


class YoloHumanTrackerNode(Node):
    def __init__(self):
        super().__init__('yolo_human_tracker')

        self.subscription = self.create_subscription(
            Yolo8Inference,
            '/Yolo8_Inference',
            self.detection_callback,
            10
        )
        
        
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            10)

        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan_filtered',
            self.laser_callback,
            10
        )
        

        self.bridge = CvBridge()
        self.image_width = None  
        self.min_distance = float('inf')
        self.distance_threshold = 3.0
        self.min_distance = 999.0

    def laser_callback(self, msg):
        self.laserscan_latest_callback = msg
        start_idx = 67
        end_idx = 133



        if start_idx < len(msg.ranges) and end_idx < len(msg.ranges):
            front_ranges = list(msg.ranges[2917:]) + list(msg.ranges[:84])


            valid_ranges = [r for r in front_ranges if not math.isnan(r) and not math.isinf(r)]

            if front_ranges:
                self.min_distance = min(front_ranges)
                print(self.min_distance)
            else:
                self.min_distance = 999.0




    def detection_callback(self, msg: Yolo8Inference):
        
        person = None
        for detection in msg.yolo8_inference:
            if detection.class_name == "person":
                person = detection
                break
        
        
        if self.image_width is None:
            self.image_width = 320

        if person is None:
            return

        cx = person.x
        image_center = self.image_width / 2
        offset_x = cx - image_center

        gain = 0.0015  
        angular_velocity = -offset_x * gain

        twist_msg = Twist()
        twist_msg.angular.z = angular_velocity

        
        if abs(offset_x) < 30 and self.min_distance > self.distance_threshold:
            twist_msg.linear.x = 0.4

        if self.min_distance < self.distance_threshold or abs(offset_x) > 30:
            twist_msg.linear.x = 0.0


        self.cmd_vel_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YoloHumanTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()