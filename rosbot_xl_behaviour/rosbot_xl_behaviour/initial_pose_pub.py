import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import quaternion_from_euler


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose',
            10
        )

        self.timer = self.create_timer(
            15.0,
            self.publish_initial_pose
        )

        self.published = False

    def publish_initial_pose(self):
        if self.published:
            self.timer.cancel()
            return
        
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'

        pose_msg.pose.pose.position.x = 1.9641932249069214
        pose_msg.pose.pose.position.y = -11.615422248840332
        pose_msg.pose.pose.position.z = 0.0501708984375
        

        roll = 0.0
        pitch = 0.0
        yaw = 1.57

        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)

        pose_msg.pose.pose.orientation.x = qx
        pose_msg.pose.pose.orientation.y = qy
        pose_msg.pose.pose.orientation.z = qz
        pose_msg.pose.pose.orientation.w = qw

        pose_msg.pose.covariance = [
            0.0025, 0,      0,      0,      0,      0,
            0,      0.0025, 0,      0,      0,      0,
            0,      0,      0.0001, 0,      0,      0,
            0,      0,      0,      0.0001, 0,      0,
            0,      0,      0,      0,      0.0001, 0,
            0,      0,      0,      0,      0,      0.0004
        ]

        self.pub.publish(pose_msg)
        self.published = True

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()