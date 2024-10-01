import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from std_msgs.msg import String
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import mediapipe as mp
import numpy as np

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
    self.publisher = self.create_publisher(String, 'MicroDreams', 10)

    self.br = CvBridge()
    
    # Used to convert between ROS and OpenCV images
    self.mp_face_mesh = mp.solutions.face_mesh
    self.face_mesh = self.mp_face_mesh.FaceMesh(max_num_faces=1)
    self.index_left_eye = [33, 160, 158, 133, 153, 144]
    self.index_right_eye = [362, 385, 387, 263, 373, 380]
    self.aux_conter = 0
    self.micro_dreams = 0
    self.blink = 0
    self.all = String()
    self.ear_thresh = 0.33
    self.separador=","
  
    

    
  def listener_callback(self, data):
        """
        Callback function.
        """
    # Display the message on the console
        self.get_logger().info(f'ACT : {self.all} ')
  
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
                    cv2.circle (frame, (x, y), 2, (0, 255, 255), 1)
                    cv2. circle (frame, (x, y), 1, (128, 0, 250) , 1)
                    #Ojo Derecho
                 for index in self.index_right_eye:
                    x = int (face_landmarks.landmark[index].x * width)
                    y = int (face_landmarks.landmark[index].y * height)
                    coordinates_right_eye.append([x,y])
                    cv2.circle (frame, (x, y), 2, (128, 0 , 250), 1)
                    cv2. circle (frame, (x, y), 1, (0, 255, 255) , 1)
        while len(coordinates_left_eye) < 6:
          coordinates_left_eye.append(1)

        d_A = np.linalg.norm(np.array(coordinates_left_eye[1]) - np.array(coordinates_left_eye[5]))
        d_B = np.linalg.norm(np.array(coordinates_left_eye[2]) - np.array(coordinates_left_eye[4]))
        d_C = np.linalg.norm(np.array(coordinates_left_eye[0]) - np.array(coordinates_left_eye[4]))
        left_eye_d = (d_A+d_B)/(2*d_C)
        
        while len(coordinates_right_eye) < 6:
          coordinates_right_eye.append(1)

        d_A = np.linalg.norm(np.array(coordinates_right_eye[1]) - np.array(coordinates_right_eye[5]))
        d_B = np.linalg.norm(np.array(coordinates_right_eye[2]) - np.array(coordinates_right_eye[4]))
        d_C = np.linalg.norm(np.array(coordinates_right_eye[0]) - np.array(coordinates_right_eye[4]))
        right_eye_d = (d_A+d_B)/(2*d_C)
        
        dual_eyes_d = (left_eye_d + right_eye_d)/2

        if dual_eyes_d < self.ear_thresh:
                 self.aux_conter +=1
        else:
            if self.aux_conter >= 1 and self.aux_conter <= 30:
                  self.aux_conter = 0
                  self.blink += 1
                  
            else:
                if self.aux_conter >=40:
                      self.micro_dreams+=1
                      self.aux_conter=0
                      

        # cv2.putText(frame, f'Blink: {int(self.blink)}', (80, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (165, 0, 0))  # Color negro
        # cv2.putText(frame, f'Micro Dreams: {int(self.micro_dreams)}', (80, 85), cv2.FONT_HERSHEY_SIMPLEX, 1, (165, 0, 0))  # Color negro
        # cv2.imshow("A", frame)
        self.all.data = str(self.blink)+self.separador+str(self.micro_dreams)
        
        self.publisher.publish(self.all)
        k = cv2.waitKey(1) & 0xFF
        
            

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