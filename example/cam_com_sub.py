#!/usr/bin/env python3

# Author: Rishiyendra

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage

class cam_sub_node():

    def __init__(self):
        # self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.camera_callback, queue_size=1, buff_size=2**24)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.camera_callback, queue_size=1, buff_size=2**24)
        rospy.on_shutdown(self.shutdown_hook)

    def camera_callback(self, data):
        np_arr = np.frombuffer(data.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if cv_image is None:
            rospy.logwarn("Empty image received")
            return

        cv2.imshow("Original", cv_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown('User pressed q')

    def shutdown_hook(self):
        rospy.loginfo("Shutting down node, closing OpenCV windows")
        cv2.destroyAllWindows()


def main():
    rospy.init_node('compressed_image_node', anonymous=True)
    compressed_image_node = cam_sub_node()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()