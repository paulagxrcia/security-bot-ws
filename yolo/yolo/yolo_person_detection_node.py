#!/usr/bin/env python3

from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from custom_msgs.msg import InferenceResult, Yolo8Inference
from std_srvs.srv import SetBool
import time

"""Este código utiliza un modelo de YOLOv8 junto con un nodo ROS2
para la detección de personas en tiempo real. Se suscribe a las imágenes
proporcionadas por la cámara del robot, procesa las imágenes para detectar
objetos, y publica los resultados."""

bridge = CvBridge()

class YoloObjectDetection(Node):
    def __init__(self) -> None:
        super().__init__('object_detection')

        #importamos el modelo preentrenado de YOLO8
        self.model = YOLO('/home/paula/security_bot_ws/src/yolo/weights/best.pt')
        self.yolo8_inference = Yolo8Inference()
        
        self.subscription = self.create_subscription(
            Image, 
            '/camera/color/image_raw',
            self.camera_callback,
            10
        )

        self.yolo8_pub = self.create_publisher(
            Yolo8Inference, 
            "/Yolo8_Inference", 
            1)
        
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)

        self.alarm_client = self.create_client(SetBool, 'toggle_alarm')

        while not self.alarm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio toggle_alarm...')

        self.person_detected_count = 0
        self.no_person_count = 0
        self.alarm_on = False

    def camera_callback(self, msg: Image) -> None:

        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(img)

        self.yolo8_inference.header.frame_id = "inference"
        self.yolo8_inference.header.stamp = self.get_clock().now().to_msg()

        person_detected = False

        for r in results:
            boxes = r.boxes
            for box in boxes:
                self.inf_result = InferenceResult()
    
                #obtener las coordenadas de la caja en el formato (arriba, izquierda, abajo, derecha)
                b = box.xyxy[0].to('cpu').detach().numpy().copy()

                """ box.xyxy contiene las coordenadas de la caja delimitadora en el formato
                (xmin, ymin, xmax, ymax), donde (xmin, ymin) es la coordenada de la 
                esquina superior izquierda de la caja,  y (xmax, ymax) la de la esquina
                inferior derecha.
                to('cpu').detach().numpy().copy() aseguran que el tensor que contiene las 
                coordenadas de la caja delimitadora se mueva a la memoria de la CPU, se 
                detache de cualquier gráfico de calculo, se convierta en un array de NumPy
                y luego se copie para asegurarse de que sea un objeto independiente en memoria.
                """

                c = box.cls

                """box.cls contiene el identificador de la clase del objeto detectado. Se utiliza
                para obtener el nombre de la clase del objeto detectado."""

                self.inf_result.class_name = self.model.names[int(c)]
                self.inf_result.left = int(b[0])
                self.inf_result.top = int(b[1])
                self.inf_result.right = int(b[2])
                self.inf_result.bottom = int(b[3])
                self.inf_result.box_width = (self.inf_result.right - self.inf_result.left)
                self.inf_result.box_height = (self.inf_result.bottom - self.inf_result.top)
                self.inf_result.x = self.inf_result.left + (self.inf_result.box_width/2.0)
                self.inf_result.y = self.inf_result.top + (self.inf_result.box_height/2.0)
                self.yolo8_inference.yolo8_inference.append(self.inf_result)

                if self.inf_result.class_name == 'person':
                    person_detected = True

            annotated_frame = results[0].plot()
            img_msg = bridge.cv2_to_imgmsg(annotated_frame)
            self.img_pub.publish(img_msg)

            self.yolo8_pub.publish(self.yolo8_inference)
            self.yolo8_inference.yolo8_inference.clear()

        # Lógica para activar/desactivar alarma
        if person_detected:
            self.person_detected_count += 1
            self.no_person_count = 0
        else:
            self.no_person_count += 1
            self.person_detected_count = 0

        if self.person_detected_count >= 3 and not self.alarm_on:
            self.call_alarm_service(True)
            self.alarm_on = True

        if self.no_person_count >= 5 and self.alarm_on:
            self.call_alarm_service(False)
            self.alarm_on = False

    def call_alarm_service(self, activate: bool):
        req = SetBool.Request()
        req.data = activate
        future = self.alarm_client.call_async(req)
        future.add_done_callback(self.alarm_response_callback)

    def alarm_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Servicio toggle_alarm respondió: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Error llamando servicio toggle_alarm: {e}')


def main(args = None) -> None:
    rclpy.init(args=args)
    object_detection = YoloObjectDetection()
    rclpy.spin(object_detection)
    object_detection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

