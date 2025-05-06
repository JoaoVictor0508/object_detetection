from datetime import datetime
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import cv2  # Para processar e exibir a imagem
from std_msgs.msg import String
import numpy as np
from scipy.optimize import minimize

import message_filters

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        self.output_file = "/home/robofei/Sambashare/teste_cenario_imu_visao_quad_maior/laser/teste.txt"

        # Subscrição dos dados do RPLidar
        self.laser_subscription_right = self.create_subscription(
            LaserScan,
            '/scan_right',  # Tópico onde os dados do RPLidar são publicados
            self.laser_callback_right,
            10
        )

        self.laser_subscription_left = self.create_subscription(
            LaserScan,
            '/scan_left',  # Tópico onde os dados do RPLidar são publicados
            self.laser_callback_left,
            100
        )

        self.horario_passado = self.get_clock().now().nanoseconds * 1e-6
        self.horario_passado_2 = self.get_clock().now().nanoseconds * 1e-6
        self.horario_passado_3 = self.get_clock().now().nanoseconds * 1e-6

        self.right_scan = None
        self.left_scan = None

        self.contador = 0

        self.image_field = None

        self.left_x = []
        self.left_y = []
        self.right_x = []
        self.right_y = []

        self.publisher = self.create_publisher(String, 'detected_object_position', 10)

    def laser_callback_right(self, msg):

        self.right_scan = msg
        # if self.left_scan is not None:
        #     self.process_lasers()

    def laser_callback_left(self, msg):

        self.left_scan = msg
        if self.right_scan is not None:
            self.process_lasers()

    def process_lasers(self):

        # print("entered process lasers")

        # horario_atual = self.get_clock().now().nanoseconds * 1e-6

        # diferenca = horario_atual - self.horario_passado_3

        # self.horario_passado_3 = horario_atual

        # print(f"Diferença de processamento: {diferenca}")

        tempo_inicial = self.get_clock().now().nanoseconds * 1e-6
        log_points = []

        img_height = 1000
        img_width = 1000
        img = np.zeros((img_height, img_width), dtype=np.uint8)

        tgt_points = []
        for i, r in enumerate(self.left_scan.ranges):
            if r < self.left_scan.range_max and np.isfinite(r):
                angle = self.left_scan.angle_min + i * self.left_scan.angle_increment
                x = r * np.cos(angle)
                y = r * np.sin(angle)

                img_x = int(x * 150*1.25) + img_width // 4
                img_y = int(-y * 150*1.25) + img_height // 2
                # img[img_y, img_x] = (0, 255, 0)  # Laser esquerdo
                img[img_y, img_x] = 255  # Laser esquerdo

                # tgt_points.append((x, y))

        log_points_2 = []

        # x_t = 3.3515645787100545
        # y_t = -0.03839459516867073
        # theta = np.radians(-0.5315020280965085)
        # scale = 0.994379940447971

        # x_t = 3.377582083114712
        # y_t = 0.018680529171434642
        # theta = np.radians(0.512361134095567)
        # scale = 0.99886227353594

        # x_t = 3.3977690079267453
        # y_t = 0.005331838233524238
        # theta = np.radians(-0.12327994859309895)
        # scale = 0.9967945899980171

        x_t = 3.381618981890454
        y_t = 0.01268125343206676
        theta = np.radians(0.2888042049976871)
        scale = 1.0050993065656602

        src_points = []
        for i, r in enumerate(self.right_scan.ranges):
            if r < self.right_scan.range_max and np.isfinite(r):
                angle = self.right_scan.angle_min + i * self.right_scan.angle_increment
                x = r * np.cos(angle)
                y = r * np.sin(angle)

                x_transformed = x * np.cos(np.radians(180.0)) - y * np.sin(np.radians(180.0))
                y_transformed = x * np.sin(np.radians(180.0)) + y * np.cos(np.radians(180.0))

                x_new = scale * (x_transformed * np.cos(theta) - y_transformed * np.sin(theta)) + x_t
                y_new = scale * (x_transformed * np.sin(theta) + y_transformed * np.cos(theta)) + y_t

                img_x = int(x_new * 150.0*1.25) + img_width // 4
                img_y = int(-y_new * 150.0*1.25) + img_height // 2

                # if 0 <= img_x < img_width and 0 <= img_y < img_height:
                    # img[img_y, img_x] = (255, 0, 0)  # Laser direito
                img[img_y, img_x] = 255  # Laser direito

                # src_points.append((x_transformed, y_transformed))
                # src_points.append((x, y))

        self.image_field = img

        # x_t, y_t, theta, scale = self.scan_matching(src_points, tgt_points)

        # print(f"Translação: x_t={x_t}, y_t={y_t}")
        # print(f"Rotação: theta={np.degrees(theta)} graus")
        # print(f"Escala: scale={scale}")

        # x_t = 3.346034883039973
        # y_t = 0.00010864852208259221
        # theta = np.radians(-0.9267436004642412)
        # scale = 1.0011016849013064

        # x_t=3.0813790149312013
        # y_t=-0.0391703562373622
        # theta=np.radians(-0.524637271310433)
        # scale=1.0009173124273694

        # x_t = 3.4155830075415485
        # y_t = -0.005734252464923715
        # theta = np.radians(0.03585109715079714)
        # scale = 1.006697709142636

        # x_t = 3.3515645787100545
        # y_t = -0.03839459516867073
        # theta = np.radians(-0.5315020280965085)
        # scale = 0.994379940447971

        # laser_right_aligned = self.transform_points(src_points, x_t, y_t, theta, scale)
        # if(self.contador % 10 == 0):
            # self.view_transformed_points(laser_right_aligned, tgt_points, 150*1.25)
        self.save_detected_image(img)

        # self.merge_lasers(tgt_points, laser_right_aligned)

        position = self.detect_robot()

        if(position is not None):
            timestamp = str(self.get_clock().now().to_msg().sec)
            with open(self.output_file, 'a') as file:
                file.write(f"{timestamp};{position[0]:.4f};{position[1]:.4f}\n")

            print(f"Detecção número: {self.contador}")
        else:
            print("Robô não detectado por nenhum dos lasers")

        self.contador += 1

        # self.right_scan = None
        # self.left_scan = None

    def view_transformed_points(self, right_points, left_points, scale):
        img_width = 1000
        img_height = 1000
        img = np.zeros((img_height, img_width, 3), dtype=np.uint8)

        # for i in range(10):
        #     img[img_height // 2 + i, img_width // 4 + 195] = (0, 255, 255)
        #     img[img_height // 2 - i, img_width // 4 + 195] = (0, 255, 255)

        for x, y in left_points:
            img_x = int(x * scale) + img_width // 4
            img_y = int(-y * scale) + img_height // 2
            # print(f"x = {x} y = {y}")
            # print(f"img_x = {img_x} img_y = {img_y}")
            img[img_y, img_x] = (0, 255, 0)  # Laser esquerdo
            # img[img_y, img_x] = 255  # Laser esquerdo

        for x, y in right_points:
            img_x = int(x * scale) + img_width // 4
            img_y = int(-y * scale) + img_height // 2
            img[img_y, img_x] = (255, 0, 0)  # Laser esquerdo
            # img[img_y, img_x] = 255  # Laser esquerdo

        img[img_height // 2, img_width // 4 + 195] = (0, 0, 255) 
        img[img_height // 2, img_width // 4 + 220] = (0, 0, 255) # 16,675cm
        img[img_height // 2, img_width // 4 + 230] = (0, 0, 255) # 23,345cm


        self.save_detected_image(img)
        

    def transform_points(self, points, x_t, y_t, theta, scale):
        transformed_points = []
        for x, y in points:
            x_new = scale * (x * np.cos(theta) - y * np.sin(theta)) + x_t
            y_new = scale * (x * np.sin(theta) + y * np.cos(theta)) + y_t
            transformed_points.append((x_new, y_new))
        
        return np.array(transformed_points)
    
    def error_function(self, params, source_points, target_points):
        x_t, y_t, theta, scale = params
        transformed_points = self.transform_points(source_points, x_t, y_t, theta, scale)

        error = 0
        for p in transformed_points:
            distances = np.linalg.norm(target_points - p, axis = 1)
            error += np.min(distances)

        print(error)
        
        return error
    
    def scan_matching(self, source_points, target_points):
        # print("Entered scan matching")

        initial_guess = [3.06, 0.0, 0.0, 1.0]
        # initial_guess = [3.24, -0.0001, np.radians(-0.1206), 0.9658]
        # initial_guess = [3.15, 0.0, 0.0, 1.0]

        # print("Entered minimize")

        result = minimize(self.error_function, 
                          initial_guess,
                          args=(source_points, target_points),
                          method="L-BFGS-B",
                          bounds=[(-5, 5), (-5, 5), (-np.pi/8, np.pi/8), (0.9, 1.10)]
        )

        print("Exited minimize")

        return result.x

    def merge_lasers(self, left_laser, right_laser):
        img_height = 1000
        img_width = 1000
        img = np.zeros((img_height, img_width), dtype=np.uint8)

        for x, y in left_laser:
            img_x = int(x * 150*1.25) + img_width // 4
            img_y = int(-y * 150*1.25) + img_height // 2
            # img[img_y, img_x] = (0, 255, 0)  # Laser esquerdo
            img[img_y, img_x] = 255  # Laser esquerdo

        # theta = np.radians(180.0)  # Ajuste fino do ângulo em graus
        # x_t = 3.23 # Deslocamento em X (metros)
        # y_t = -0.025   # Deslocamento em Y (metros)

        for x, y in right_laser:
            img_x = int(x * 150.0*1.25) + img_width // 4
            img_y = int(-y * 150.0*1.25) + img_height // 2

            if 0 <= img_x < img_width and 0 <= img_y < img_height:
                # img[img_y, img_x] = (255, 0, 0)  # Laser direito
                img[img_y, img_x] = 255  # Laser direito

        # self.save_detected_image(img)

        self.image_field = img
        self.left_scan = None
        self.right_scan = None

    def detect_robot(self):

        # x_max = 700 * 1.25
        # x_min = 220 * 0.75
        # y_max = 640 * 1.25
        # y_min = 160 * 0.75

        x_max = 820
        x_min = 255
        y_max = 640 * 1.25
        y_min = 180

        img = self.image_field

        # Aplicar uma dilatação leve para conectar pontos próximos
        kernel = np.ones((3, 3), np.uint8)

        # img = cv2.dilate(img, kernel, iterations=2)

        # img = cv2.erode(img, kernel, iterations=1)

        # self.save_detected_image(img)

        blurred_img = cv2.GaussianBlur(img, (3, 3), 0)  # Suavizar a imagem

        # self.save_detected_image(blurred_img)

        # Converta para RGB para permitir a coloração dos círculos
        img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        # Aplicando a Transformada de Hough para detecção de círculos
        circles = cv2.HoughCircles(
            blurred_img, 
            cv2.HOUGH_GRADIENT, dp=1.2, minDist=100,
            param1=100, param2=5, minRadius=14, maxRadius=19
        )

        if circles is not None:

            horario_atual = self.get_clock().now().nanoseconds * 1e-6

            # diferenca = horario_atual - self.horario_passado

            # self.horario_passado = horario_atual

            # print(diferenca)

            circles = np.uint16(np.around(circles))
            refined_circles = []

            for circle in circles[0, :]:
                x, y, radius = circle

                if(x_min < x < x_max) and (y_min < y < y_max):
                    refined_circles.append(circle)

            if refined_circles is not None:
                for (x, y, r) in refined_circles:
                    # Destacar o semi-círculo com uma cor diferente (vermelho)
                    cv2.circle(img_color, (x, y), r, (0, 0, 255), 4)  # Círculo vermelho

                    # Mostrar o centro do círculo
                    cv2.circle(img_color, (x, y), 2, (0, 255, 0), 3)  # Centro verde

                    # cv2.circle(img_color, (200, 400), r, (0, 0, 255), 4)
                    cv2.circle(img_color, (200, 400), 2, (255, 0, 0), 3)

                    center = (1000/4, 1000/2)

                    scale = 150.0*1.25

                    x_relativo = (x - center[0]) / scale
                    y_relativo = (y - center[1]) / scale

                    erro = 0.0
                    erro_y= 0.0
                    
                    x_relativo = -1.66 + (x - center[0]) / scale + erro
                    y_relativo = -(y - center[1]) / scale + erro_y

                    # self.get_logger().info(f"X imagem: {x}, Y Imagem {y}")
                    # self.get_logger().info(f"Left = {left} Centro Real: x = {x_real:.4f}, y = {y_real:.4f}, r = {r}")
                    # self.get_logger().info(f"Centro Real: x = {x_relativo:.5f}, y = {y_relativo:.5f}, r = {r}")

                    # self.save_detected_image(img_color)

                    return(x_relativo, y_relativo)
            else:
                return None


    def detect_robot_circle(self, points_x, points_y, left):
        """
        Detecta a posição do círculo (robô) a partir dos pontos do laser.
        Retorna as coordenadas do robô no espaço do mundo real (em metros).
        """
        # Gerar a imagem binária com os pontos (x, y) do RPLidar
        image_size = 800  # Aumentar a resolução para 800x800 pixels
        if(left == True):
            center = (image_size // 4, image_size // 2)  # Centro da imagem (onde o laser está)
        else:
            center = (image_size // 8, image_size // 2)

        # Normalizando os dados de (x, y) para a imagem
        scale = image_size / (0.5*self.right_scan.range_max)   # Ajustar a escala conforme a área de leitura

        x_img = np.int32(center[0] + points_x * scale)
        y_img = np.int32(center[1] - points_y * scale)  # Inverter o eixo y para imagens (coordenada inversa)

        # if(left == True):
        #     x_max = 630
        #     x_min = 130
        #     y_max = 620
        #     y_min = 200
        # elif(left == False):
        #     x_max = 630
        #     x_min = 130
        #     y_max = 600
        #     y_min = 180

        if(left == True):
            x_max = 630
            x_min = 130
            y_max = 630
            y_min = 200
        elif(left == False):
            x_max = 630
            x_min = 130
            y_max = 600
            y_min = 260

        # Criar a imagem binária em tons de cinza e depois converter para RGB
        img = np.zeros((image_size, image_size), dtype=np.uint8)
        for xi, yi in zip(x_img, y_img):
            if 0 <= xi < image_size and 0 <= yi < image_size:
                img[yi, xi] = 255  # Definir o ponto de laser como branco (255)

        # Aplicar uma dilatação leve para conectar pontos próximos
        kernel = np.ones((3, 3), np.uint8)

        img = cv2.dilate(img, kernel, iterations=2)

        img = cv2.erode(img, kernel, iterations=1)

        self.save_detected_image(img)

        # self.get_logger().info("Nova Imagem")

        blurred_img = cv2.GaussianBlur(img, (3, 3), 0)  # Suavizar a imagem

        # self.save_detected_image(blurred_img)

        # Converta para RGB para permitir a coloração dos círculos
        img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        # Aplicando a Transformada de Hough para detecção de círculos
        circles = cv2.HoughCircles(
            blurred_img, 
            cv2.HOUGH_GRADIENT, dp=1.0, minDist=100,
            param1=50, param2=5, minRadius=13, maxRadius=19
        )

        if circles is not None:
            circles = np.uint16(np.around(circles))
            refined_circles = []

            for circle in circles[0, :]:
                x, y, radius = circle

                if(x_min < x < x_max) and (y_min < y < y_max):
                    refined_circles.append(circle)

            if refined_circles is not None:
                for (x, y, r) in refined_circles:
                    # Destacar o semi-círculo com uma cor diferente (vermelho)
                    cv2.circle(img_color, (x, y), r, (0, 0, 255), 4)  # Círculo vermelho

                    # Mostrar o centro do círculo
                    cv2.circle(img_color, (x, y), 2, (0, 255, 0), 3)  # Centro verde
                    
                    x_relativo = (x - center[0]) / scale
                    y_relativo = (y - center[1]) / scale

                    self.get_logger().info(f"Left = {left} Centro Relativo: x = {x_relativo:.4f}, y = {y_relativo:.4f}, r = {r}")

                    if(left == True):
                        x_real = -1.36 + (x - center[0]) / scale
                        y_real = - (y - center[1]) / scale
                    elif(left == False):
                        x_real = 1.98 - (x - center[0]) / scale
                        y_real = -0.6 + (y - center[1]) / scale
                    self.get_logger().info(f"X imagem: {x}, Y Imagem {y}")
                    self.get_logger().info(f"Left = {left} Centro Real: x = {x_real:.4f}, y = {y_real:.4f}, r = {r}")

                    self.save_detected_image(img_color)

                    return(x_real, y_real)
            else:
                return None
    
    def calculate_robot_position(self, left_points, right_points):

        # Combinar as posições
        if left_points and right_points:
            # Média das coordenadas

            if(left_points[0] > 0.4 and right_points[0] > 0.4):
                x_avg = right_points[0] * 0.9 + left_points[0] * 0.1
                y_avg = right_points[1] * 0.9 + left_points[1] * 0.1
            elif(left_points[0] < -0.4 and right_points[0] < -0.4):
                x_avg = right_points[0] * 0.1 + left_points[0] * 0.9
                y_avg = right_points[1] * 0.1 + left_points[1] * 0.9
            else:
                x_avg = (left_points[0] + right_points[0]) / 2
                y_avg = (left_points[1] + right_points[1]) / 2
            return (x_avg, y_avg)
        elif left_points:
            return left_points  # Apenas o laser esquerdo detectou
        elif right_points:
            return right_points  # Apenas o laser direito detectou
        else:
            return None  # Nenhum dos lasers detectou

    def laser_callback(self, msg):
        # Convertendo as leituras polares (distância, ângulo) para coordenadas cartesianas (x, y)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        x = msg.ranges * np.cos(angles)
        y = msg.ranges * np.sin(angles)

        # Detectar o semi-círculo
        self.detect_semi_circle(x, y, msg.range_max)

    def detect_semi_circle(self, img):
        # # Gerar a imagem binária com os pontos (x, y) do RPLidar
        # image_size = 800  # Aumentar a resolução para 800x800 pixels
        # center = (image_size // 2, image_size // 2)  # Centro da imagem (onde o laser está)

        # # Normalizando os dados de (x, y) para a imagem
        # scale = image_size / (0.5*range_max)   # Ajustar a escala conforme a área de leitura
        # x_img = np.int32(center[0] + x * scale)
        # y_img = np.int32(center[1] - y * scale)  # Inverter o eixo y para imagens (coordenada inversa)

        # # Criar a imagem binária em tons de cinza e depois converter para RGB
        # img = np.zeros((image_size, image_size), dtype=np.uint8)
        # for xi, yi in zip(x_img, y_img):
        #     if 0 <= xi < image_size and 0 <= yi < image_size:
        #         img[yi, xi] = 255  # Definir o ponto de laser como branco (255)

        # Aplicar uma dilatação leve para conectar pontos próximos
        kernel = np.ones((3, 3), np.uint8)

        img = cv2.dilate(img, kernel, iterations=1)

        img = cv2.erode(img, kernel, iterations=1)

        # self.save_detected_image(img)

        self.get_logger().info("Nova Imagem")

        # blurred_img = cv2.GaussianBlur(img, (5, 5), 0)  # Suavizar a imagem

        # self.save_detected_image(blurred_img)

        # Converta para RGB para permitir a coloração dos círculos
        img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        # Aplicando a Transformada de Hough para detecção de círculos
        circles = cv2.HoughCircles(
            img, 
            cv2.HOUGH_GRADIENT, dp=1.0, minDist=100,
            param1=80, param2=5, minRadius=10, maxRadius=15
        )

        if circles is not None:
            # print("Círculos encontrados")
            circles = np.uint16(np.around(circles))
            refined_circles = []

            for circle in circles[0, :]:
                x, y, radius = circle

                # if(130 < x < 660) and (180 < y < 620):
                refined_circles.append(circle)

            if refined_circles is not None:
                # print("Círculos refinados encontrados")
                for (x, y, r) in refined_circles:
                    # Destacar o semi-círculo com uma cor diferente (vermelho)
                    cv2.circle(img_color, (x, y), r, (0, 0, 255), 4)  # Círculo vermelho

                    # Mostrar o centro do círculo
                    cv2.circle(img_color, (x, y), 2, (0, 255, 0), 3)  # Centro verde

                    # x_real = (x - center[0]) / scale
                    # y_real = (y - center[1]) / scale
                    # self.get_logger().info(f"X imagem: {x}, Y Imagem {y}")
                    # self.get_logger().info(f"Centro: x = {x_real:.4f}, y = {y_real:.4f}, r = {r}")

                    # timestamp = str(self.get_clock().now().to_msg().sec)
                    # with open(self.output_file, 'a') as file:
                    #     file.write(f"{timestamp};{x_real:.4f};{y_real:.4f}\n")

                # Salvar a imagem colorida quando um semi-círculo for detectado
                self.save_detected_image(img_color)


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
