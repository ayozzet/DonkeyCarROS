#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage

class ImageProcessor:
    def __init__(self):
        self.bridge_object = CvBridge()
        self.frame = None #initialize frame attribute
        # self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.camera_callback)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.camera_callback)

    def camera_callback(self, camera):
        try:
            np_arr = np.frombuffer(camera.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            frame = cv2.resize(cv_image,(1280,960))
            self.frame  = frame
            # frame = cv2.resize(cv_image,(640,480))
        except CvBridgeError as e:
            print(e)
            rospy.signal_shutdown('Quit')
            cv2.destroyAllWindows()
            return
            # frame = cv2.resize(cv_image,(640,480))
            # hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


class TrackbarManager:
    def __init__(self , image_processor):
        self.image_processor = image_processor
        self.nothing = lambda x: None
        cv2.namedWindow("Trackbars")
        cv2.createTrackbar("L - H", "Trackbars", 0, 179, self.nothing)
        cv2.createTrackbar("L - S", "Trackbars", 0, 255, self.nothing)
        cv2.createTrackbar("L - V", "Trackbars", 0, 255, self.nothing)
        cv2.createTrackbar("U - H", "Trackbars", 179, 179, self.nothing)
        cv2.createTrackbar("U - S", "Trackbars", 255, 255, self.nothing)
        cv2.createTrackbar("U - V", "Trackbars", 255, 255, self.nothing)

        self.process_trackbars()

    def process_trackbars(self):
        while True:
            if self.image_processor.frame is None:
                continue  # Wait until the frame is available

            frame = self.image_processor.frame

            l_h = cv2.getTrackbarPos("L - H", "Trackbars")
            l_s = cv2.getTrackbarPos("L - S", "Trackbars")
            l_v = cv2.getTrackbarPos("L - V", "Trackbars")
            u_h = cv2.getTrackbarPos("U - H", "Trackbars")
            u_s = cv2.getTrackbarPos("U - S", "Trackbars")
            u_v = cv2.getTrackbarPos("U - V", "Trackbars")

            lower_range = np.array([l_h, l_s, l_v])
            upper_range = np.array([u_h, u_s, u_v])

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower_range, upper_range)
            res = cv2.bitwise_and(frame, frame, mask=mask)
            mask_3 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            stacked = np.hstack((mask_3,frame,res))

            # if stacked.shape[0] > 0 and stacked.shape[1] > 0:
            #     print("syiall: Stacked Image Has Valid Shape")
            # else:
            #     print("mantop: Stacked Image Has Empty Shape (Check Image Processing)")
            #     return
            # # print("banaa")
            try:
                # print("uhhhh")
                cv2.imshow("Trackbars",cv2.resize(stacked,None,fx=0.4,fy=0.4))
                # print("ahhhh")
                # cv2.imshow("Trackbars" , stacked)
                # cv2.imshow("SYIBALL", cv_image)
                # print("ada5")
            except Exception as e:
                print("Exception at cv2.imshow:", e)
                if cv2.getWindowProperty("Trackbars", cv2.WND_PROP_VISIBLE) < 0:
                    print("** Error: Trackbars window might be hidden or minimized. Please check your desktop environment. **")

            # print("sebelum")
            key = cv2.waitKey(1)
            # print("selepas")
            if key == 27:
            #     print("cv2.waitKey() did not receive any key event.")
            # else:
            #     print("key tekan", key)

            # # If the user presses `s` then print this array.
                if key == ord('s'):
                    thearray = [[l_h,l_s,l_v],[u_h, u_s, u_v]]
                    print(thearray)
                # print("ada6")
                # cv2.imshow( "test", frame)

                # Also save this array as penval.npy
                np.save('hsv_value',thearray)
                # break
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.signal_shutdown('Quit')
                cv2.destroyAllWindows()

def main():
    rospy.init_node('huzzza', anonymous=True)
    image_processor = ImageProcessor()
    trackbar_manager = TrackbarManager(image_processor)    
    rate = rospy.Rate(5)
    ctrl_c = False

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Shutting Down")
        rospy.signal_shutdown('Quit')
        cv2.destroyAllWindows()
        pass
        # print("ada7")


if __name__ == '__main__':
    main()

