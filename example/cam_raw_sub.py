#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class cam_sub_node:

    def __init__(self):
        self.bridge = CvBridge()

        # self.image_sub = rospy.Subscriber("/raspicam_node/image", Image, self.camera_callback)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.camera_callback)

        rospy.on_shutdown(self.shutdown_hook)

    def camera_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr(f"cv_bridge error: {e}")
            return

        cv2.imshow("Original", cv_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown('User pressed q')

    def shutdown_hook(self):
        rospy.loginfo("Shutting down node, closing OpenCV windows")
        cv2.destroyAllWindows()


def main():
    rospy.init_node('raw_image_node', anonymous=True)
    raw_image_node = cam_sub_node()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()