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
	def __init__(self, controller, focus_mode: str = "manual", lens_position:float = 3.4):
		self.controller = controller
		
		self.picam2 = Picamera2()

		# Flipping resolution doesn't work
		self.img_size = (1280, 960)
		# second frame 10mm below first frame
		self.m_frame_distance = (10, 0, 0)
		self.lens_position = lens_position
		self.focus_mode=focus_mode

		self.configure_cameras()

	def configure_cameras(self):
		# === Initialize cameras ===
		self.cam_left = Picamera2(1)
		self.cam_right = Picamera2(0)

		self.config_left = self.cam_left.create_preview_configuration(main={"size": self.img_size})
		self.config_right = self.cam_right.create_preview_configuration(main={"size": self.img_size})

		self.config_left["AfMode"] = controls.AfModeEnum.Manual
		self.config_right["AfMode"] = controls.AfModeEnum.Manual

		self.cam_left.configure(self.config_left)
		self.cam_right.configure(self.config_right)

		self.cam_left.set_controls({"LensPosition": self.lens_position})
		self.cam_right.set_controls({"LensPosition": self.lens_position})

		self.cam_left.start()
		self.cam_right.start()

	# Get the focus mode
	def get_focus_mode(self):
		return (self.cam_left.capture_metadata(), self.cam_right.capture_metadata())

	def take_photo(self, id: int = 0):
		if not id:
			self.cam_left.start()

			# Capture one image with the default configurations.
			image = self.cam_left.capture_array("main")
		else:
			self.cam_right.start()

			# Capture one image with the default configurations.
			image = self.cam_right.capture_array("main")

		return image

	def get_imgs_for_depth(self, write_log):
		# Take both photos
		frame_left = self.cam_left.capture_array()
		frame_right = self.cam_right.capture_array()

		logging.write_log("server", "Return from image_stream get depth images")
		return np.flip(frame_left), np.flip(frame_right)

	def __del__(self):
		self.cam_left.close()
		self.cam_right.close()
