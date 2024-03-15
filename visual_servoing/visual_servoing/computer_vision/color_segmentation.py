import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################


def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def cd_color_segmentation(img, template):
	"""
	Implement the cone detection using color segmentation algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected. BGR.
		template_file_path; Not required, but can optionally be used to automate setting hue filter values.
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
	"""
	########## YOUR CODE STARTS HERE ##########

	# setting thresholds manually

	ORANGE_THRESHOLD = ([10, 150, 152], [50, 255, 255])


	# convert bgr to hsv
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

	# set lower and upper bounds for cone
	lower_bound = np.array(ORANGE_THRESHOLD[0])
	upper_bound = np.array(ORANGE_THRESHOLD[1])

	# using template to set filter values

	# template_image = cv2.imread(template)
	# template_hsv = cv2.cvtColor(template_image, cv2.COLOR_BGR2HSV)

	# hue_values = template_hsv[:, :, 0]

	# average_hue_value = np.mean(hue_values)

	# lower_bound = average_hue_value - 10
	# upper_bound = average_hue_value + 10

	mask = cv2.inRange(hsv, lower_bound, upper_bound)
	# print(f"Mask: {mask}")

	# find contours from masks
	contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	# print(f"Contours: {contours}")

	 # If we have contours
	if len(contours) != 0:

		# Find the biggest countour by area
		c = max(contours, key = cv2.contourArea)

		# Get a bounding rectangle around that contour
		x, y, w, h = cv2.boundingRect(c)

		cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)
		cv2.circle(img, (int(x+w/2), y+h), radius=0, color=(0, 0, 255), thickness=-1)
		bounding_box = ((x,y),(x+w,y+h))
		bottom_left = (x,y+h)

		return bounding_box[0],bounding_box[1],bottom_left
	return None,None,None
