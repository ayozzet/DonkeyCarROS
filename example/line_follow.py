#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import cv2
import numpy as np

class line_follower():
        
    def __init__(self):
        # self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.camera_callback, queue_size=1, buff_size=2**24)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.camera_callback, queue_size=1, buff_size=2**24)
        rospy.on_shutdown(self.shutdown_hook)

    def camera_callback(self, data):
        np_arr = np.frombuffer(data.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        resize = cv2.resize(cv_image, (640, 480))

        hsv = cv2.cvtColor(resize, cv2.COLOR_BGR2HSV)

        low = np.array([12, 15, 178])
        high = np.array([22, 40, 255])
        mask1 = cv2.inRange(hsv, low, high)
        blur = cv2.GaussianBlur(mask1, (15,15), 0)

        edge = cv2.Canny(blur, 100, 200)
        pix = np.asarray(edge)
        get = np.where(pix[360] == 255)
        index = np.asarray(get[0])

        if index.size > 0 :
            index_left = index[0]
            index_right = index[-1]
            center_ori = int((index_right - index_left)/2 + index_left)
            
            offset = -150 
            center = center_ori + offset
            
            cv2.circle(resize, (center, 360), 5, (250,0,0), -2)

            cv2.circle(resize, (center_ori, 360), 5, (0,255,0), -2)

        cv2.line(resize, (0, 360),(640, 360), (0, 0, 255), 1)

        cv2.imshow("frame", resize)
    
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown('User pressed q')

    def shutdown_hook(self):
        rospy.loginfo("Shutting down node, closing OpenCV windows")
        cv2.destroyAllWindows()


def main():
    rospy.init_node('line_following_node', anonymous=True)
    line_following_node = line_follower()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()