import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from copy import deepcopy
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

        self.security_route = [
            [0.5845, -11.4295],
            [1.3253, 4.8699],
            [-3.4750, 4.8107],
            [-3.4750, -11.3110]
        ]

        self.route_poses = []

        self.timer_period = 0.5  # segundos
        self.timer = self.create_timer(self.timer_period, self.patrol_loop)

    def start_patrol_callback(self, request, response):
        if self.patrolling:
            response.success = False
            response.message = 'Ya se está ejecutando una patrulla.'
            return response

        self.get_logger().info('🚶 Iniciando patrulla de seguridad...')
        self.patrolling = True

        self.navigator = BasicNavigator()
        self.get_logger().info('⏳ Esperando a que Nav2 esté activo...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('✅ Nav2 está activo.')

        self.route_poses = []
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.orientation.w = 1.0

        for pt in self.security_route:
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            pose.header.stamp = self.navigator.get_clock().now().to_msg()
            self.route_poses.append(deepcopy(pose))

        # Enviar toda la ruta con goThroughPoses
        self.navigator.goThroughPoses(self.route_poses)

        response.success = True
        response.message = 'Patrulla iniciada.'
        return response

    def stop_patrol_callback(self, request, response):
        if not self.patrolling:
            response.success = False
            response.message = 'No hay patrulla en ejecución.'
        else:
            self.get_logger().warn('⚠️ Deteniendo patrulla manualmente...')
            self.patrolling = False

            if self.navigator is not None:
                self.navigator.cancelTask()

            response.success = True
            response.message = 'Patrulla cancelada.'
        return response

    def patrol_loop(self):
        if not self.patrolling or self.navigator is None:
            return

        if self.navigator.isTaskComplete():
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('✅ Patrulla completada correctamente.')
            elif result == TaskResult.CANCELED:
                self.get_logger().info('⛔ Patrulla cancelada.')
            else:
                self.get_logger().error('❌ Patrulla fallida.')

            self.patrolling = False

def main(args=None):
    rclpy.init(args=args)
    node = SecurityPatrolNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
