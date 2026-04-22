#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
import apriltag
import os
import threading

os.environ["QT_QPA_PLATFORM"] = "xcb"

class AprilTagDetector:
    def __init__(self):

        self.latest_frame = None
        self.lock = threading.Lock()
        self.new_frame_event = threading.Event()

        # self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.camera_callback, queue_size=1, buff_size=2**24)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.camera_callback, queue_size=1, buff_size=2**24)

        options = apriltag.DetectorOptions(families="tag36h11")
        self.detector = apriltag.Detector(options)

        # ✅ Detection runs in its own thread — never blocks the subscriber
        self.process_thread = threading.Thread(target=self.process_loop, daemon=True)
        self.process_thread.start()

        rospy.on_shutdown(self.shutdown_hook)

    def camera_callback(self, data):
        # ✅ Just decode and store — done in microseconds
        np_arr = np.frombuffer(data.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        with self.lock:
            self.latest_frame = frame  # always overwrite with newest

        self.new_frame_event.set()  # signal processing thread

    def process_loop(self):
        while not rospy.is_shutdown():
            # ✅ Wait until a new frame arrives
            self.new_frame_event.wait(timeout=1.0)
            self.new_frame_event.clear()

            with self.lock:
                if self.latest_frame is None:
                    continue
                frame = self.latest_frame.copy()

            # ✅ All heavy work here, off the subscriber thread
            frame = cv2.resize(frame, (640, 480))
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detections = self.detector.detect(gray)

            for det in detections:

                cX, cY = int(det.center[0]), int(det.center[1])
                cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)
                cv2.putText(frame, str(det.tag_id),
                            (cX, cY - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (255, 0, 0), 2)

            cv2.imshow("AprilTag Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.signal_shutdown('User pressed q')

    def shutdown_hook(self):
        rospy.loginfo("Shutting down node, closing OpenCV windows")
        cv2.destroyAllWindows()


def main():
    rospy.init_node('apriltag_node')
    node = AprilTagDetector()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()