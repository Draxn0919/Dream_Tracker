import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2

class ImagenDisplay(Node):
    def __init__(self):
        super().__init__('image_display')

        self.subscription = self.create_subscription(Image, 'video_publish_frame', self.listener_callback, 10)
        self.subscription2 = self.create_subscription(String, 'tracking_eyes', self.listener_callback2, 10)
        self.subscription3 = self.create_subscription(String, 'Emotion', self.listener_callback3, 10)
        self.subscription4 = self.create_subscription(String, 'MicroDreams', self.listener_callback4, 10)
        

        self.bridge = CvBridge()
        self.mirada = ""
        self.emo = ""
        self.blink = ""
        self.micro_dreams = ""
        self.all = ""
        self.anger_count=0
        self.sadness_count=0
        self.count_right=0
        self.count_left=0
        self.alerta=""
        self.micro_dreams_count=0

    def listener_callback(self, data):
        try:
            # Convertir ROS Image a OpenCV image
            frame = self.bridge.imgmsg_to_cv2(data)
            
            # Mostrar la información de la mirada
            cv2.putText(frame, f'Mirada: {self.mirada}', (30, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (165, 0, 0)) 
            cv2.putText(frame, f'Emocion: {self.emo}', (30, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (165, 0, 0)) 
            cv2.putText(frame, f'Micro Dreams: {self.micro_dreams}', (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (165, 0, 0))
            cv2.putText(frame, f'Parpadeos: {self.blink}', (30, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (165, 0, 0))
            cv2.putText(frame, f'{self.alerta}', (30,240), cv2.FONT_HERSHEY_SIMPLEX, 1, (165, 0, 0))
            
            # Mostrar la imagen
            cv2.imshow("Visualizador", frame)
            
            # Manejar eventos de teclas (por ejemplo, cerrar ventana con 'q')
            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                cv2.destroyAllWindows()
        except Exception as e:
            self.get_logger().error(f'Error al procesar la imagen: {str(e)}')

    def listener_callback2(self, msg):
        self.mirada = msg.data
        self.get_logger().info(f'Recibido String 1: {self.mirada}')
        if self.mirada == "Mirada a la derecha":
            self.count_right+=1
        if self.mirada == "Mirada a la izquierda":
            self.count_left+=1

        

        if self.count_left + self.count_right > 3:
            self.alerta="Concentrate en la ruta"
            print(self.alerta)
        
    
    def listener_callback3(self, msg):
        self.emo = msg.data
        self.get_logger().info(f'Recibido String 2: {self.emo}')

        if self.emo  == 'Enojado':
            self.anger_count += 1
        elif self.emo  == 'Triste':
            self.sadness_count += 1
        
        if self.anger_count + self.sadness_count > 3:
            self.alerta="Llamar a servicios de \n emergencia o a alguien \n de confianza."
            print(self.alerta)
            
                
    
    def listener_callback4(self, msg):
        self.all = msg.data
        self.all= self.all.split(",")
        self.blink=self.all[0]
        self.micro_dreams=self.all[1]
        self.get_logger().info(f'Recibido String 3: {self.micro_dreams}')
        self.micro_dreams_count =int(self.micro_dreams)
        if self.micro_dreams_count> 2:
            self.alerta="Te recomiendo descansar"
            print(self.alerta)
    
    
    

def main(args=None):
    rclpy.init(args=args)
    image_display = ImagenDisplay()
    rclpy.spin(image_display)
    image_display.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
