import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from copy import deepcopy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class ShelfInspectionNode(Node):
    def __init__(self):
        super().__init__('inspection_service')

        self.inspecting = False
        self.navigator = None

        self.start_srv = self.create_service(Trigger, '/start_inspection', self.inspection_callback)
        self.stop_srv = self.create_service(Trigger, '/stop_inspection', self.stop_callback)

        self.get_logger().info('Servicios /start_inspection y /stop_inspection disponibles.')

        # Datos de estanterías
        self.shelves = [
            [[5.78, -0.04], [2, -0.04]],
            [[2, -2.14], [5.78, -2.14], [2, -2.14]],
            [[2, -3.85], [5.78, -3.85], [2, -3.85]],
            [[2, -5.75], [5.78, -5.75], [2, -5.75]],
            [[2, -7.65], [5.78, -7.65], [2, -7.65]],
            [[2, -9.8], [5.78, -9.8], [2, -9.8]],
            [[2, -11.6], [5.78, -11.6], [2, -11.6]]
        ]
        self.abc = ['A', 'B', 'C', 'D', 'E', 'F', 'G']
        self.start_pose_coords = [2, -0.04]

        # Estado para control de ruta
        self.current_shelf_idx = 0
        self.current_point_idx = 0
        self.route_in_progress = False

        # Timer para loop no bloqueante
        self.timer_period = 0.5  # segundos
        self.timer = self.create_timer(self.timer_period, self.patrol_loop)

        # Pose reusable
        self.pose = PoseStamped()
        self.pose.header.frame_id = 'map'
        self.pose.pose.orientation.w = 1.0

    def inspection_callback(self, request, response):
        if self.inspecting:
            response.success = False
            response.message = 'Ya se está ejecutando una inspección.'
            return response

        self.get_logger().info('🔍 Iniciando inspección de estanterías...')
        self.inspecting = True
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        # Reset indices y estado
        self.current_shelf_idx = 0
        self.current_point_idx = 0
        self.route_in_progress = False

        # Enviar primera posición (start_pose)
        self.goTo(self.start_pose_coords)
        self.route_in_progress = True

        response.success = True
        response.message = 'Inspección iniciada.'
        return response

    def stop_callback(self, request, response):
        if not self.inspecting:
            response.success = False
            response.message = 'No hay inspección en ejecución.'
        else:
            self.get_logger().warn('⚠️ Cancelando inspección manualmente...')
            self.inspecting = False
            self.route_in_progress = False
            if self.navigator is not None:
                self.navigator.cancelTask()
            response.success = True
            response.message = 'Inspección cancelada.'
        return response

    def goTo(self, coords):
        self.pose.pose.position.x = float(coords[0])
        self.pose.pose.position.y = float(coords[1])
        self.pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.navigator.goToPose(deepcopy(self.pose))

    def patrol_loop(self):
        # Si no estamos inspeccionando, nada que hacer
        if not self.inspecting or not self.route_in_progress or self.navigator is None:
            return

        # Si la tarea está completa, avanzamos a siguiente punto o shelf
        if self.navigator.isTaskComplete():
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                # Si estábamos yendo al punto inicial, empezar con la inspección
                if self.current_shelf_idx == 0 and self.current_point_idx == 0 and self.route_in_progress:
                    self.get_logger().info('✅ Posición inicial alcanzada. Comenzando inspección...')

                # Avanzar al siguiente punto
                shelf = self.shelves[self.current_shelf_idx]

                # Si estamos justo después de llegar a start_pose, avanzar a primer punto estantería A
                if self.current_shelf_idx == 0 and self.current_point_idx == 0:
                    self.current_point_idx = 0
                else:
                    self.current_point_idx += 1

                if self.current_point_idx < len(shelf):
                    self.get_logger().info(f'➡️ Moviéndonos a punto {self.current_point_idx + 1} de shelf {self.abc[self.current_shelf_idx]}')
                    self.goTo(shelf[self.current_point_idx])
                else:
                    self.get_logger().info(f'✅ Shelf {self.abc[self.current_shelf_idx]} inspeccionada.')
                    self.current_shelf_idx += 1
                    self.current_point_idx = 0
                    if self.current_shelf_idx < len(self.shelves):
                        self.get_logger().info(f'➡️ Moviéndonos a shelf {self.abc[self.current_shelf_idx]}')
                        self.goTo(self.shelves[self.current_shelf_idx][self.current_point_idx])
                    else:
                        self.get_logger().info('✅ Inspección completada correctamente.')
                        self.inspecting = False
                        self.route_in_progress = False
            else:
                self.get_logger().warn('⚠️ Navegación fallida o cancelada.')
                self.inspecting = False
                self.route_in_progress = False

def main(args=None):
    rclpy.init(args=args)
    node = ShelfInspectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
