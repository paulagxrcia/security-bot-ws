import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from copy import deepcopy
import time
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class ShelfInspectionNode(Node):
    def __init__(self):
        super().__init__('inspection_service')
        self.inspecting = False
        self.navigator = None

        self.start_srv = self.create_service(Trigger, '/start_inspection', self.inspection_callback)
        self.stop_srv = self.create_service(Trigger, '/stop_inspection', self.stop_callback)

        self.get_logger().info('Servicios /start_inspection y /stop_inspection disponibles.')

    def inspection_callback(self, request, response):
        if self.inspecting:
            response.success = False
            response.message = 'Ya se está ejecutando una inspección.'
            return response

        self.get_logger().info('🔍 Iniciando inspección de estanterías...')
        self.inspecting = True
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        Shelf_A = [[5.78, -0.04], [2, -0.04]]
        Shelf_B = [[2, -2.14], [5.78, -2.14], [2, -2.14]]
        Shelf_C = [[2, -3.85], [5.78, -3.85], [2, -3.85]]
        Shelf_D = [[2, -5.75], [5.78, -5.75], [2, -5.75]]
        Shelf_E = [[2, -7.65], [5.78, -7.65], [2, -7.65]]
        Shelf_F = [[2, -9.8], [5.78, -9.8], [2, -9.8]]
        Shelf_G = [[2, -11.6], [5.78, -11.6],[2, -11.6]]
        shelves = [Shelf_A, Shelf_B, Shelf_C, Shelf_D, Shelf_E, Shelf_F, Shelf_G]
        abc = ['A', 'B', 'C', 'D', 'E', 'F', 'G']

        start_pose = [2, -0.04]
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.orientation.w = 1.0

        def goTo(coords):
            pose.pose.position.x = float(coords[0])
            pose.pose.position.y = float(coords[1])
            pose.header.stamp = self.navigator.get_clock().now().to_msg()
            self.navigator.goToPose(deepcopy(pose))

        # Ir a posición inicial
        goTo(start_pose)
        if not self._wait_until_complete():
            response.success = False
            response.message = 'Cancelado antes de llegar al punto inicial.'
            return response

        for i, shelf in enumerate(shelves):
            for pt in shelf:
                goTo(pt)
                if not self._wait_until_complete():
                    self.navigator.cancelTask()
                    goTo(start_pose)
                    self._wait_until_complete()
                    response.success = False
                    response.message = f'Inspección cancelada durante shelf {abc[i]}.'
                    self.inspecting = False
                    return response
            self.get_logger().info(f'✅ Shelf {abc[i]} inspeccionado.')

        self.inspecting = False
        response.success = True
        response.message = 'Inspección completada correctamente.'
        return response

    def stop_callback(self, request, response):
        if not self.inspecting:
            response.success = False
            response.message = 'No hay inspección en ejecución.'
        else:
            self.inspecting = False
            response.success = True
            response.message = '🛑 Cancelando inspección...'
        return response

    def _wait_until_complete(self):
        while not self.navigator.isTaskComplete():
            if not self.inspecting:
                return False
            time.sleep(0.5)
        return True


def main(args=None):
    rclpy.init(args=args)
    node = ShelfInspectionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
