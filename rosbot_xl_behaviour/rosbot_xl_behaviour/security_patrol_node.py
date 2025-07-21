import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from copy import deepcopy
import time
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


class SecurityPatrolNode(Node):
    def __init__(self):
        super().__init__('patrolling_service')

        self.patrolling = False
        self.navigator = None

        self.start_srv = self.create_service(
            Trigger, 
            '/start_patrolling', 
            self.start_patrol_callback)

        self.stop_srv = self.create_service(
            Trigger,
            '/stop_patrolling',
            self.stop_patrol_callback)

        self.get_logger().info('Servicios /start_patrolling y /stop_patrolling listos.')

    def start_patrol_callback(self, request, response):
        if self.patrolling:
            response.success = False
            response.message = 'Ya se está ejecutando una patrulla.'
            return response

        self.get_logger().info('🚶 Iniciando patrulla de seguridad...')
        self.patrolling = True

        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        security_route = [
            [0.5845, -11.4295],
            [1.3253, 4.8699],
            [-3.4750, 4.8107],
            [-3.4750, -11.3110]
        ]

        route_poses = []
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.orientation.w = 1.0

        for pt in security_route:
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            pose.header.stamp = self.navigator.get_clock().now().to_msg()
            route_poses.append(deepcopy(pose))

        self.navigator.goThroughPoses(route_poses)

        while not self.navigator.isTaskComplete():
            if not self.patrolling:
                self.get_logger().warn('⚠️ Patrullaje cancelado por el usuario.')
                self.navigator.cancelTask()
                while not self.navigator.isTaskComplete():
                    time.sleep(0.5)
                response.success = False
                response.message = 'Patrulla cancelada manualmente.'
                return response
            time.sleep(0.5)

        result = self.navigator.getResult()
        if result != TaskResult.SUCCEEDED:
            response.success = False
            response.message = 'La patrulla falló o fue cancelada.'
        else:
            response.success = True
            response.message = 'Patrulla completada correctamente.'

        self.patrolling = False
        return response

    def stop_patrol_callback(self, request, response):
        if not self.patrolling:
            response.success = False
            response.message = 'No hay patrulla en ejecución.'
        else:
            self.patrolling = False
            response.success = True
            response.message = 'Cancelando patrulla en curso...'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SecurityPatrolNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

