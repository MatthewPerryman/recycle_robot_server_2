#import cv2  # sudo apt-get install python-opencv
import numpy as np
from ctypes import *
import sys
from time import sleep, time
from utils import logging
from libcamera import controls

try:
	from picamera2 import Picamera2
except:
	sys.exit(0)

class ImageStream:
	def __init__(self, controller):
		self.controller = controller
		
		self.picam2 = Picamera2()

		# Flipping resolution doesn't work
		self.resolution = (640, 480, 3)
		# second frame 10mm below first frame
		self.m_frame_distance = (10, 0, 0)
	
	# Set the focus mode
	def set_focus_mode(self, focus_mode, focus_value=None):
		if focus_mode == "Continuous":
			self.picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})
		elif focus_mode == "Manual":
			self.picam2.set_controls({"AfMode": controls.AfModeEnum.Manual})
			if focus_value is None:
				self.picam2.set_controls({"LensPosition": 5.6818181818})
			else:
				self.picam2.set_controls({"LensPosition": focus_value})

	# Get the focus mode
	def get_focus_mode(self):
		return self.picam2.capture_metadata()

	def take_photo(self):
		self.picam2.start()

		# Capture one image with the default configurations.
		image = self.picam2.capture_array("main")

		return image

	def get_imgs_for_depth(self, write_log):
		# Capture image 1
		logging.write_log("server", "First Photo")
		img1 = self.take_photo()

		# Move the robot right 10mm
		logging.write_log("server", "Move Arm 1")
		self.controller.move_by_vector(self.m_frame_distance)

		# Capture image 2
		logging.write_log("server", "Second Photo")
		img2 = self.take_photo()

		# Reset position
		logging.write_log("server", "Move Arm 2")
		arm_move_function(self.m_frame_distance, reverse_vector=True)

		logging.write_log("server", "Return from image_stream get depth images")
		return np.flip(img1), np.flip(img2)

	def __init__(self):
		# open camera
		preview_config = self.picam2.create_still_configuration(main={"size": (640, 480)})
		self.picam2.configure(preview_config)

		self.picam2.set_controls({"AfMode": controls.AfModeEnum.Manual})
#		self.picam2.set_controls({"LensPosition": 5.6818181818})

		self.picam2.set_controls({"LensPosition": 10})

	def __del__(self):
		self.picam2.close()
