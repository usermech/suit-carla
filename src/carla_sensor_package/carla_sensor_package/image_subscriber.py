import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np 

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.declare_parameter('camera_id', '0')
        camera_id = self.get_parameter('camera_id').value
        self.get_logger().info(f"Displayed camera index: {camera_id}")
        topic_name =  '/carla/image_' + str(camera_id)
        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Display the image using OpenCV
        cv2.imshow("CARLA Camera View", cv_image)
        
        # Display the image until a key is pressed
        cv2.waitKey(1)
def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    
    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    
    # Close OpenCV window and shutdown node
    cv2.destroyAllWindows()
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

