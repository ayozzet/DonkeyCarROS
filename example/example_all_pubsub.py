#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
from geometry_msgs.msg import Twist
import numpy as np

class AV:

    def __init__(self):
        self.ctrl_c = False
        self.image_sub = rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, self.image_callback, queue_size=2000)
        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)    

    def publish_once_in_cmd_vel(self, cmd):        
        while not self.ctrl_c:
            connections = self.pub_cmd_vel.get_num_connections()
            if connections > 0:
                self.pub_cmd_vel.publish(cmd)
                # rospy.loginfo("Publish in cmd_vel...")
                break
            else:
                self.rate.sleep()

    def stop_car(self):
        # rospy.loginfo("Stopping...")
        self.move_msg.linear.x = 0.0
        self.move_msg.angular.z = 0.0
        self.publish_once_in_cmd_vel(self.move_msg)
        # time.sleep(0.3)  
    
    def car_fwd(self):
        # rospy.loginfo("Stopping...")
        self.move_msg.linear.x = 0.4
        self.move_msg.angular.z = 0.0
        self.publish_once_in_cmd_vel(self.move_msg)
      
    def image_callback(self,image_sub):
        
        try:
            self.pub_cmd_vel = rospy.Publisher('/cmd_vel',Twist, queue_size=1)
            self.move_msg = Twist()
            # image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
            np_arr = np.frombuffer(image_sub.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)
        cv_image = cv2.resize(cv_image, (820,616)) 
        # print(cv_image.shape)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        self.car_fwd()
        self.move_msg.linear.x = 0.35
        self.publish_once_in_cmd_vel(self.move_msg)


        cv2.imshow("Original", cv_image)
        cv2.imshow("HSV", hsv)

        if cv2.waitKey(1) & 0xFF==ord('q'):
            self.stop_car() 
            rospy.signal_shutdown('Quit')
            cv2.destroyAllWindows()      
        
def main():
    rospy.init_node('AV_node', anonymous=True)
    AV_node = AV()
    # image_sub = rospy.Subscriber('/raspicam_node/image/compressed',Image, AV.image_callback)
    try:
        # np_arr = np.frombuffer(image_sub.data, np.uint8)
        # cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # AV_node.rqtimage_callback()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Shutting down")
        pass
    #cv2.destroyAllWindows()
    
if __name__ == "__main__":
    main()