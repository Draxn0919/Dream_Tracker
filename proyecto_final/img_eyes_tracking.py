import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from std_msgs.msg import String
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import mediapipe as mp
import numpy as np

class ImagenTracking(Node):
  """
  Create an ImagenTracking class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_tracking')
       
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(Image, 'video_publish_frame', self.listener_callback, 10)
    self.subscription # prevent unused variable warning
    self.publisher = self.create_publisher(String, 'tracking_eyes', 10)
    self.br = CvBridge()
    
    # Used to convert between ROS and OpenCV images
    self.mp_face_mesh = mp.solutions.face_mesh
    self.face_mesh = self.mp_face_mesh.FaceMesh(max_num_faces=1)
    self.index_left_eye = [130,243,27,23]
    self.index_right_eye = [463,359,253,257]
    self.mirada = String()
    
    

    
  def listener_callback(self, data):
        """
        Callback function.
        """
    # Display the message on the console
        
  
    # Convert ROS Image message to OpenCV image
        frame = self.br.imgmsg_to_cv2(data)
        frame = cv2.flip(frame, 1)
        height, width, _ = frame.shape
        frame_rgb = cv2.cvtColor (frame, cv2. COLOR_BGR2RGB)
        results = self.face_mesh. process (frame_rgb)
        coordinates_left_eye=[]
        coordinates_right_eye=[]
        if results.multi_face_landmarks is not None:
          for face_landmarks in results. multi_face_landmarks:
                 ##Ojo Izquierdo
                 for index in self.index_left_eye:
                    x = int (face_landmarks.landmark[index].x * width)
                    y = int (face_landmarks.landmark[index].y * height)
                    coordinates_left_eye.append([x,y])
                    # cv2.circle (frame, (x, y), 2, (0, 255, 255), 1)
                    # cv2. circle (frame, (x, y), 1, (128, 0, 250) , 1)
                    #Ojo Derecho
                 for index in self.index_right_eye:
                    x = int (face_landmarks.landmark[index].x * width)
                    y = int (face_landmarks.landmark[index].y * height)
                    coordinates_right_eye.append([x,y])
                    # cv2.circle (frame, (x, y), 2, (128, 0 , 250), 1)
                    # cv2. circle (frame, (x, y), 1, (0, 255, 255) , 1)

              
        while len(coordinates_right_eye) < 4:
          coordinates_right_eye.append((1,1))
          coordinates_right_eye.append((1,1))
          coordinates_right_eye.append((1,1))
          coordinates_right_eye.append((1,1))
        while len(coordinates_left_eye) < 4:
          coordinates_left_eye.append((1,1))
          coordinates_left_eye.append((1,1))
          coordinates_left_eye.append((1,1))
          coordinates_left_eye.append((1,1))
        
        d_R = np.linalg.norm(np.array(coordinates_right_eye[0]) - np.array(coordinates_right_eye[1]))
        d_L = np.linalg.norm(np.array(coordinates_left_eye[0]) - np.array(coordinates_left_eye[1]))


        if d_R < (1/1.5*d_L):
          self.mirada.data = "Mirada a la derecha"
        if d_L < (1/1.05*d_R):
          self.mirada.data = "Mirada centrada"
        if d_L < (1/1.5*d_R):
          self.mirada.data = "Mirada a la izquierda"

        
        self.publisher.publish(self.mirada)
        
        
                      

        # # cv2.line(frame, coordinates_left_eye[2], coordinates_left_eye[3], (0, 255, 0), thickness=2)
        # cv2.line(frame, coordinates_left_eye[0], coordinates_left_eye[1], (0, 255, 0), thickness=2)
        # # cv2.line(frame, coordinates_right_eye[2], coordinates_right_eye[3], (0, 255, 0), thickness=2)
        # # cv2.line(frame, coordinates_right_eye[0], coordinates_right_eye[1], (0, 255, 0), thickness=2)
        # cv2.putText(frame, f'Mirada: {self.mirada.data}', (80, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (165, 0, 0)) 
        # cv2.imshow("A", frame)
        k = cv2.waitKey(1) & 0xFF
        
            

    # Se crea la función de dibujo de la malla
    

    # Mostrar el video con la malla facial procesada

   
def main(args=None):
   
  # Initialize the rclpy library
  rclpy.init(args=args)
   
  # Create the node
  image_tracking = ImagenTracking()
   
  # Spin the node so the callback function is called.
  rclpy.spin(image_tracking)
   
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_tracking.destroy_node()
   
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
   
if __name__ == '__main__':
  main()