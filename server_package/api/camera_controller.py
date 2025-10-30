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
	picam2 = Picamera2()

	# Flipping resolution doesn't work
	resolution = (640, 480, 3)
	# second frame 10mm below first frame
	camera_separation_mm_polar = (10, 0, 0)
	reversed_camera_separation_mm = (-10, 0, 0)
	
	# Set the focus mode
	def set_focus_mode(self, focus_mode, focus_value=None):
		self.picam2.start()
		sleep(1)

		if focus_value is None:
			focus_value = 5.6818181818  # default middle value

		if focus_mode == "Continuous":
			self.picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous, "ExposureTime": 10000, "AnalogueGain": 1.0})
			

			if self.picam2.camera_controls['AfMode'] != controls.AfModeEnum.Continuous:
				logging.write_log("server", "Warning: Continuous focus mode not set correctly")
			else:
				logging.write_log("server", "Focus mode set to Continuous")
		elif focus_mode == "Manual":
			self.picam2.set_controls({"AfMode": controls.AfModeEnum.Manual})
			self.picam2.set_controls({"LensPosition": focus_value})

			if self.picam2.camera_controls['AfMode'] != controls.AfModeEnum.Manual:
				logging.write_log("server", "Warning: Manual focus mode not set correctly")
			else:
				logging.write_log("server", "Focus mode set to Manual")
				
			if self.picam2.camera_controls['LensPosition'] != focus_value:
				logging.write_log("server", "Warning: Manual focus value not set correctly")
			else:
				logging.write_log("server", f"Focus value set to {focus_value}")
		else:
			logging.write_log("server", "Warning: Focus mode not recognized")
		
		print("Focus Mode Set to ", focus_mode
			  , " with value ", self.picam2.camera_controls['LensPosition'])
		
		sleep(2)

	# Get the focus mode
	def get_focus_mode(self):
		return self.picam2.capture_metadata()

	def take_photo(self):
		self.picam2.start()

		# Capture one image with the default configurations.
		image = self.picam2.capture_array("main")

		return image

	def get_imgs_for_depth(self, arm_move_function):
		self.set_focus_mode("Manual")

		# Capture image 1
		logging.write_log("server", "First Photo")
		img1 = self.take_photo()

		# Move the robot right 10mm
		logging.write_log("server", "Move Arm 1")
		arm_move_function(self.camera_separation_mm_polar)

		# Capture image 2
		logging.write_log("server", "Second Photo")
		img2 = self.take_photo()

		# Reset position
		logging.write_log("server", "Move Arm 2")
		arm_move_function(self.reversed_camera_separation_mm)

		logging.write_log("server", "Return from image_stream get depth images")
		return np.flip(img1), np.flip(img2), self.picam2.capture_metadata()['LensPosition']

	def __init__(self):
		# open camera
		preview_config = self.picam2.create_still_configuration(main={"size": (640, 480)})
		self.picam2.configure(preview_config)

		self.picam2.set_controls({"AfMode": controls.AfModeEnum.Manual})

		self.picam2.set_controls({"LensPosition": 10})

	def __del__(self):
		self.picam2.close()
