import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from custom_msgs.msg import Yolo8Inference
import time

class IntruderRecorderNode(Node):
    def __init__(self):
        super().__init__('intruder_recorder_node')

        self.subscription = self.create_subscription(
            Yolo8Inference,
            '/Yolo8_Inference',
            self.detection_callback,
            10)

        self.recording_client = self.create_client(
            SetBool, 
            'toggle_recording')
        
        while not self.recording_client.wait_for_service(timeout_sec=1.0):
            pass

        self.recording = False

        self.last_msg_time = time.time()
        self.no_msg_timeout = 3.0  # segundos sin mensajes para parar grabación

        self.person_detected_count = 0  # contador para confirmación de persona detectada
        self.no_person_count = 0        # contador para confirmación de ausencia de persona

        self.timer = self.create_timer(1.0, self.check_no_detection)

    def detection_callback(self, msg: Yolo8Inference):
        self.last_msg_time = time.time()  # actualiza timestamp

        person_detected = any(det.class_name == 'person' for det in msg.yolo8_inference)

        if person_detected:
            self.person_detected_count += 1
            self.no_person_count = 0
        else:
            self.no_person_count += 1
            self.person_detected_count = 0

        # Confirmar detección solo si se detecta persona 3 veces seguidas
        if self.person_detected_count >= 3:
            if not self.recording:
                self.get_logger().info('Intruso confirmado tras 3 detecciones seguidas, iniciando grabación...')
                self.call_recording_service(True)
                self.recording = True

        # Confirmar ausencia solo si no se detecta persona 5 veces seguidas
        if self.no_person_count >= 5 and self.recording:
            self.get_logger().info('No se detecta persona durante 5 ciclos, deteniendo grabación...')
            self.call_recording_service(False)
            self.recording = False

    def check_no_detection(self):
        elapsed = time.time() - self.last_msg_time
        if elapsed > self.no_msg_timeout:
            if self.recording:
                self.get_logger().info('No se reciben mensajes de detección, deteniendo grabación...')
                self.call_recording_service(False)
                self.recording = False

    def call_recording_service(self, start: bool):
        req = SetBool.Request()
        req.data = start
        future = self.recording_client.call_async(req)
        future.add_done_callback(self.service_response_callback)


    def service_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Servicio toggle_recording respondió: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Error llamando servicio toggle_recording: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = IntruderRecorderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
