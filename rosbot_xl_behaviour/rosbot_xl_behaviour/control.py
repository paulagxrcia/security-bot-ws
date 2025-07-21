import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger, SetBool
import sys
import tty
import termios
import select
import os

instructions = """
Control de Modo de Seguridad
----------------------------
Presiona una tecla para ejecutar una acción:

    1 : Iniciar patrullaje
    2 : Iniciar inspección de estanterías
    S : Detener patrullaje
    I : Detener inspección de estanterías
    R : Detener grabación
    A : Desactivar alarma
    Q : Salir

Esperando entrada...
"""

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        self.start_patrol_client = self.create_client(Trigger, '/start_patrolling')
        self.stop_patrol_client = self.create_client(Trigger, '/stop_patrolling')
        self.shelf_inspect_client = self.create_client(Trigger, '/start_inspection')
        self.stop_inspect_client = self.create_client(Trigger, '/stop_inspection')
        self.toggle_recording_client = self.create_client(SetBool, '/toggle_recording')
        self.alarm_client = self.create_client(SetBool, '/toggle_alarm')

        self.wait_for_services()

        self.last_message = ""

        self.print_interface()

        # Guarda configuración terminal para restaurar después
        self.old_term_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        # Timer que chequea teclado cada 0.1 seg
        self.timer = self.create_timer(0.1, self.check_keyboard)

    def wait_for_services(self):
        self.get_logger().info('Esperando servicios...')
        clients = [
            (self.start_patrol_client, '/start_patrolling'),
            (self.stop_patrol_client, '/stop_patrolling'),
            (self.shelf_inspect_client, '/start_inspection'),
            (self.stop_inspect_client, '/stop_inspection'),
            (self.toggle_recording_client, '/toggle_recording'),
            (self.alarm_client, '/toggle_alarm'),
        ]
        for client, name in clients:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f'Servicio {name} no disponible aún...')

    def print_interface(self):
        os.system('clear')
        print(instructions)
        if self.last_message:
            print("\n" + self.last_message)

    def check_keyboard(self):
        if self.is_key_pressed():
            key = sys.stdin.read(1).lower()
            self.handle_key(key)

    def is_key_pressed(self):
        dr, _, _ = select.select([sys.stdin], [], [], 0)
        return dr != []

    def handle_key(self, key):
        if key == '1':
            self.call_trigger_service(self.start_patrol_client, "▶️ Iniciando patrullaje...")
        elif key == '2':
            self.call_trigger_service(self.shelf_inspect_client, "📦 Iniciando inspección de estanterías...")
        elif key == 's':
            self.call_trigger_service(self.stop_patrol_client, "⛔ Patrullaje detenido.")
        elif key == 'i':
            self.call_trigger_service(self.stop_inspect_client, "⛔ Inspección de estanterías detenida.")
        elif key == 'r':
            self.call_toggle_recording_service(False, "⏹️ Grabación detenida.")
        elif key == 'a':
            self.call_alarm_service(False, "🔕 Alarma desactivada.")
        elif key == 'q':
            self.get_logger().info("👋 Saliendo del control...")
            rclpy.shutdown()
            return
        else:
            self.last_message = f"[!] Tecla no válida: {key}"
            self.print_interface()
            return

        self.last_message = ""
        self.print_interface()

    def call_trigger_service(self, client, log_msg):
        request = Trigger.Request()
        future = client.call_async(request)
        future.add_done_callback(lambda f: self.get_logger().info(log_msg))

    def call_toggle_recording_service(self, state: bool, log_msg: str):
        request = SetBool.Request()
        request.data = state
        future = self.toggle_recording_client.call_async(request)
        future.add_done_callback(lambda f: self.get_logger().info(log_msg))

    def call_alarm_service(self, state: bool, log_msg: str):
        request = SetBool.Request()
        request.data = state
        future = self.alarm_client.call_async(request)
        future.add_done_callback(lambda f: self.get_logger().info(log_msg))

    def __del__(self):
        # Restaurar terminal al cerrar nodo
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_term_settings)


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
