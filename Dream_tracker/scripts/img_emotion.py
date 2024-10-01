import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from std_msgs.msg import String
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import mediapipe as mp
import numpy as np
import math

class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
       
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(Image, 'video_publish_frame', self.listener_callback, 10)
    self.subscription # prevent unused variable warning

    self.publisher = self.create_publisher(String, 'Emotion', 10)

    self.br = CvBridge()
    
    # Used to convert between ROS and OpenCV images
    self.mp_face_mesh = mp.solutions.face_mesh
    self.face_mesh = self.mp_face_mesh.FaceMesh(max_num_faces=1)
    self.index_left_eyebrow = [65,158]
    self.index_right_eyebrow = [295, 385]
    self.index_lips = [78, 308, 13, 14]
    self.animo = String()
    

    
  def listener_callback(self, data):
        """
        Callback function.
        """
    # Display the message on the console
        self.get_logger().info('ACT')
  
    # Convert ROS Image message to OpenCV image
        frame = self.br.imgmsg_to_cv2(data)
        frame = cv2.flip(frame, 1)
        height, width, _ = frame.shape
        frame_rgb = cv2.cvtColor (frame, cv2. COLOR_BGR2RGB)
        results = self.face_mesh. process (frame_rgb)
        coordinates_left_eyebrow=[]
        coordinates_right_eyebrow=[]
        coordinates_lips=[]
        if results.multi_face_landmarks is not None:
          for face_landmarks in results. multi_face_landmarks:
                 ##Ojo Izquierdo
                 for index in self.index_left_eyebrow:
                    x = int (face_landmarks.landmark[index].x * width)
                    y = int (face_landmarks.landmark[index].y * height)
                    coordinates_left_eyebrow.append([x,y])
                    cv2.circle (frame, (x, y), 2, (0, 255, 255), 1)
                    cv2. circle (frame, (x, y), 1, (128, 0, 250) , 1)
                    #Ojo Derecho
                 for index in self.index_right_eyebrow:
                    x = int (face_landmarks.landmark[index].x * width)
                    y = int (face_landmarks.landmark[index].y * height)
                    coordinates_right_eyebrow.append([x,y])
                    cv2.circle (frame, (x, y), 2, (128, 0 , 250), 1)
                    cv2. circle (frame, (x, y), 1, (0, 255, 255) , 1)
                    #Boca
                 for index in self.index_lips:
                    x = int (face_landmarks.landmark[index].x * width)
                    y = int (face_landmarks.landmark[index].y * height)
                    coordinates_lips.append([x,y])
                    cv2.circle (frame, (x, y), 2, (128, 0 , 250), 1)
                    cv2. circle (frame, (x, y), 1, (0, 255, 255) , 1)
        while len(coordinates_left_eyebrow) < 2:
          coordinates_left_eyebrow.append(1)
        
        while len(coordinates_lips) < 4:
          coordinates_lips.append(1)
        
        while len(coordinates_right_eyebrow) < 2:
          coordinates_right_eyebrow.append(1)
        
        d_eyesbrow_left = np.linalg.norm(np.array(coordinates_left_eyebrow[1])-np.array(coordinates_left_eyebrow[0]))
        d_eyesbrow_left = np.linalg.norm(np.array(coordinates_right_eyebrow[1])-np.array(coordinates_right_eyebrow[0]))
        d_eyesbrow_dual = (d_eyesbrow_left+d_eyesbrow_left)/2
        d_lips_opening = np.linalg.norm(np.array(coordinates_lips[1])-np.array(coordinates_lips[0]))
        d_lips_extremes = np.linalg.norm(np.array(coordinates_lips[3])-np.array(coordinates_lips[2]))
        
        if d_eyesbrow_dual > 27 and d_lips_extremes > 15 and d_lips_opening > 80:
           self.animo.data="alegre"
        if d_eyesbrow_dual > 30 and d_lips_extremes > 25 and 65 < d_lips_opening < 78:
           self.animo.data= "Sorprendido"
        if 27 < d_eyesbrow_dual < 30 and 0 <= d_lips_extremes < 5 and 65 < d_lips_opening < 78:
           self.animo.data="Triste"
        if 22 < d_eyesbrow_dual < 27 and 0 <= d_lips_extremes < 5 and 60 < d_lips_opening < 67:
           self.animo.data = "neutro"
        if 15 < d_eyesbrow_dual < 22 and 0 <= d_lips_extremes < 5 and 60 < d_lips_opening < 67:
           self.animo.data = "Enojado"
        if d_eyesbrow_dual == 0 and d_lips_extremes == 0 and d_lips_opening == 0:
           self.animo.data = "No se encontro usuario"
      #   cv2.putText(frame, f'Animo: {self.animo.data}', (40, 75), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255),1)
      #   cv2.imshow("A", frame)
        k = cv2.waitKey(1) & 0xFF
        self.publisher.publish(self.animo)
        
            

    # Se crea la función de dibujo de la malla
    

    # Mostrar el video con la malla facial procesada
    
   
def main(args=None):
   
  # Initialize the rclpy library
  rclpy.init(args=args)
   
  # Create the node
  image_subscriber = ImageSubscriber()
   
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
   
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
   
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
   
if __name__ == '__main__':
  main()