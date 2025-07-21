import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import threading
import time

class AlarmController(Node):
    def __init__(self):
        super().__init__('alarm_controller')

        # Creamos el servicio que otros nodos pueden usar para activar o desactivar la alarma
        self.srv = self.create_service(
            SetBool, 
            'toggle_alarm', 
            self.handle_alarm
        )

        self.alarm_active = False
        self.alarm_thread = None


    def handle_alarm(self, request, response):
        if request.data:
            if not self.alarm_active:
                self.alarm_active = True
                self.alarm_thread = threading.Thread(target=self.run_alarm)
                self.alarm_thread.start()
            response.success = True
            response.message = 'Alarma activada'
        else:
            if self.alarm_active:
                self.get_logger().info('🔕 Desactivando alarma...')
                self.alarm_active = False
            response.success = True
            response.message = 'Alarma desactivada'
        return response

    def run_alarm(self):
        self.get_logger().warn('🔴 ALERTA. INTRUSO DETECTADO.')

        while self.alarm_active:
            self.get_logger().warn('🔴 ALERTA. INTRUSO DETECTADO.')
            time.sleep(2)


def main(args=None):
    rclpy.init(args=args)
    node = AlarmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Nodo detenido por el usuario.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
