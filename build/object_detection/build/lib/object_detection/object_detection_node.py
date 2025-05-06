from datetime import datetime
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import cv2  # Para processar e exibir a imagem
from std_msgs.msg import String

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        self.output_file = "/home/robofei/ros2_ws/src/object_detection/logs/test.txt"

        # Subscrição dos dados do RPLidar
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',  # Tópico onde os dados do RPLidar são publicados
            self.laser_callback,
            10
        )

        self.publisher = self.create_publisher(String, 'detected_object_position', 10)

    def laser_callback(self, msg):
        # Convertendo as leituras polares (distância, ângulo) para coordenadas cartesianas (x, y)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        x = msg.ranges * np.cos(angles)
        y = msg.ranges * np.sin(angles)

        # Detectar o semi-círculo
        self.detect_semi_circle(x, y, msg.range_max)

    def detect_semi_circle(self, x, y, range_max):
        # Gerar a imagem binária com os pontos (x, y) do RPLidar
        image_size = 800  # Aumentar a resolução para 800x800 pixels
        center = (image_size // 2, image_size // 2)  # Centro da imagem (onde o laser está)

        # Normalizando os dados de (x, y) para a imagem
        scale = image_size / (0.5*range_max)   # Ajustar a escala conforme a área de leitura
        x_img = np.int32(center[0] + x * scale)
        y_img = np.int32(center[1] - y * scale)  # Inverter o eixo y para imagens (coordenada inversa)

        # Criar a imagem binária em tons de cinza e depois converter para RGB
        img = np.zeros((image_size, image_size), dtype=np.uint8)
        for xi, yi in zip(x_img, y_img):
            if 0 <= xi < image_size and 0 <= yi < image_size:
                img[yi, xi] = 255  # Definir o ponto de laser como branco (255)

        # Aplicar uma dilatação leve para conectar pontos próximos
        kernel = np.ones((3, 3), np.uint8)

        img = cv2.dilate(img, kernel, iterations=2)

        img = cv2.erode(img, kernel, iterations=1)

        self.get_logger().info("Nova Imagem")

        blurred_img = cv2.GaussianBlur(img, (5, 5), 0)  # Suavizar a imagem

        # Converta para RGB para permitir a coloração dos círculos
        img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        # Aplicando a Transformada de Hough para detecção de círculos
        circles = cv2.HoughCircles(
            img, 
            cv2.HOUGH_GRADIENT, dp=1.0, minDist=100,
            param1=80, param2=5, minRadius=11, maxRadius=15
        )

        if circles is not None:
            circles = np.uint16(np.around(circles))
            refined_circles = []

            for circle in circles[0, :]:
                x, y, radius = circle

                if(130 < x < 660) and (180 < y < 620):
                    refined_circles.append(circle)

            if refined_circles is not None:
                for (x, y, r) in refined_circles:
                    # Destacar o semi-círculo com uma cor diferente (vermelho)
                    cv2.circle(img_color, (x, y), r, (0, 0, 255), 4)  # Círculo vermelho

                    # Mostrar o centro do círculo
                    cv2.circle(img_color, (x, y), 2, (0, 255, 0), 3)  # Centro verde

                    x_real = (x - center[0]) / scale
                    y_real = (y - center[1]) / scale
                    # self.get_logger().info(f"X imagem: {x}, Y Imagem {y}")
                    self.get_logger().info(f"Centro: x = {x_real:.4f}, y = {y_real:.4f}, r = {r}")

                    timestamp = str(self.get_clock().now().to_msg().sec)
                    with open(self.output_file, 'a') as file:
                        file.write(f"{timestamp};{x_real:.4f};{y_real:.4f}\n")

                # Salvar a imagem colorida quando um semi-círculo for detectado
                # self.save_detected_image(img_color)


    def save_detected_image(self, img):
        # Cria um diretório para salvar as imagens, se não existir
        output_dir = "/home/robofei/ros2_ws/src/object_detection/images"
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        # # Nome do arquivo com timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S%f")
        filename = os.path.join(output_dir, f"semi_circle_{timestamp}.png")

        # Nome do arquivo com timestamp
        # filename = os.path.join(output_dir, f"semi_circle_{name}.png")
        
        # Salva a imagem com o nome gerado
        cv2.imwrite(filename, img)
        # self.get_logger().info(f"Imagem salva: {filename}") 

    def publish_position(self, x, y):
        msg = String()
        msg.data = f"Centro detectado: ({x:.2f}, {y:.2f}) metros"
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

    # Fechar todas as janelas do OpenCV ao final
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
