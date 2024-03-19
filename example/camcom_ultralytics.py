#!usr/bin/python3

#Author by Rishiyendra 

import rospy
import cv2
import numpy as np
from std_msgs.msg import Empty
from sensor_msgs.msg import CompressedImage

from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator, colors

model = YOLO("best.pt")
names = model.model.names

class DonkeyCar(object):

	def __init__(self):
		self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.camera_callback)
		#self.image_sub = rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.camera_callback)
		self.ctrl_c = False

	def camera_callback(self, data):

		np_arr = np.frombuffer(data.data, np.uint8)
		cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

		result = model.track(cv_image)
		annotated_frame = result[0].plot()

		for box in result[0].boxes:
			class_id = result[0].names[box.cls[0].item()]
			cords = box.xyxy[0].tolist()
			cords = [round(x) for x in cords]
			conf = round(box.conf[0].item(), 2)
			width = cords[2] - cords[0]
			height = cords[3] - cords[1]
			print("Object type:", class_id)
			print("Coordinates:", cords)
			print("Probability:", conf)
			print("Width:", width, "Height:", height)
			print("---")

			if (class_id == "Parking Sign"):
				print("suki suki daisuki")

		else:
				pass

		cv2.imshow("Original", cv_image)
		cv2.imshow("With Detections", annotated_frame)

		if cv2.waitKey(1) & 0xFF == ord('q'):
			rospy.signal_shutdown('Quit')
			cv2.destroyAllWindows()


def main():
	rospy.init_node('donkey_node', anonymous=True)
	drone_image_node = DonkeyCar()
	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("Shutting down")
		pass

	while not ctrl_c:
		rate.sleep()

if __name__ == '__main__':
	main()
