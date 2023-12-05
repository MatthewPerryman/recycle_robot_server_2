#import cv2  # sudo apt-get install python-opencv
import numpy as np
from ctypes import *
import sys
from time import sleep, time
from Utils import Logging
from libcamera import controls

try:
	from picamera2 import Picamera2
except:
	sys.exit(0)


# Stock functions except __init__, __del__, and updated take_photo
# def laplacian(img):
# 	img_gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
# 	img_sobel = cv2.Laplacian(img_gray, cv2.CV_16U)
# 	return cv2.mean(img_sobel)[0]


class ImageStream:
	# load arducam shared object file
	arducam_vcm = CDLL('./Server_Package/PiCode/rpiWebServer/API/lib/libarducam_vcm.so')
	picam2 = Picamera2()

	# Flipping resolution doesn't work
	resolution = (640, 480, 3)
	# second frame 10mm below first frame
	m_frame_distance = (10, 0, 0)

	def adjust_lens(self, val):
		self.arducam_vcm.vcm_write(val)
	
	
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

	def calculation(self, camera):
		rawCapture = PiRGBArray(camera)
		self.camera.capture(rawCapture, format="bgr", use_video_port=True)
		image = rawCapture.array
		rawCapture.truncate(0)
		return image

	# Runs through all lense focal lengths until clarity reduces, then sets length to best length
	def focus(self):
		print("Start focusing")

		self.best_index = 10
		self.best_clarity = 0.0
		self.last_value = 0.0
		self.poor_f_len_count = 0
		self.focal_length = 10

		while True:
			# Adjust focus
			self.adjust_lens(self.focal_length)
			# Take image and calculate image clarity
			val = self.calculation(self.camera)
			# Find the maximum image clarity
			if val > self.best_clarity:
				self.best_index = self.focal_length
				self.best_clarity = val
				self.poor_f_len_count = 0
			elif val < self.last_value:  # If the image clarity starts to decrease
				self.poor_f_len_count += 1
				# Image clarity is reduced by six consecutive frames
				if self.poor_f_len_count > 6:
					self.focal_length = self.best_index
					break

				self.last_value = val

			# Increase the focal distance
			self.focal_length += 15
			if self.focal_length > 1000:
				self.focal_length = self.best_index
				break

		# Adjust focus to the best
		self.adjust_lens(self.focal_length)
		print('Focused')

	def take_photo(self):
		# Create image array
		# image_array = np.empty(self.resolution, dtype=np.uint8)
		self.picam2.start()

		# Capture image to np array
		# self.camera.capture(image_array, 'rgb')

		# Capture one image with the default configurations.
		image = self.picam2.capture_array("main")

		#self.picam2.stop()

		return image

	# # Returns a captured photo
	# # Does not refocus before shot
	# def take_simple_photo(self):
	# 	self.cam_open()
	# 	image_array = self.capture_photo()
	# 	self.camera.close()

	# 	# Returns image in numpy array format.
	# 	return image_array

	# def take_focused_photo(self):
	# 	self.cam_open()
	# 	self.focus()
	# 	image_array = self.capture_photo()
	# 	self.camera.close()

		# return image_array, self.focal_length

	def get_imgs_for_depth(self, arm_move_function, write_log):
		Logging.write_log("server", "Open camera")
		#self.cam_open()

		#sleep(1)
		# Capture image 1
		Logging.write_log("server", "First Photo")
		img1 = self.take_photo()

		# Get the f_len of the first image
		#focal_len = self.focal_length

		# Move the robot right 10mm
		Logging.write_log("server", "Move Arm 1")
		arm_move_function(self.m_frame_distance)

		# Refocus for new location
		Logging.write_log("server", "Focus Camera in New Location")
		#self.focus()

		#sleep(1)
		# Capture image 2
		Logging.write_log("server", "Second Photo")
		img2 = self.take_photo()

		# Reset focus for position 1
		Logging.write_log("server", "Move Arm 2")
		#self.focal_length = focal_len
		arm_move_function(self.m_frame_distance, reverse_vector=True)

		#self.camera.close()

		Logging.write_log("server", "Return from image_stream get depth images")
		return np.flip(img1), np.flip(img2), self.picam2.capture_metadata()['LensPosition']

	# def cam_open(self):
	# 	# Open camera - Set camera resolution to 480x640(Small resolution for faster speeds.)
	# 	# Camera needs a flipped resolution
	# 	self.camera = picamera.PiCamera(resolution=(self.resolution[1], self.resolution[0]))

	# 	# self.camera.shutter_speed = 30000

	# 	# https://picamera.readthedocs.io/en/release-1.13/recipes1.html?highlight=shutter%20speed#capturing-consistent-images
	# 	self.camera.iso = 100
	# 	# Wait for the automatic gain control to settle
	# 	sleep(2)

	# 	self.camera.shutter_speed = self.camera.exposure_speed
	# 	# To fix exposure gains, let analog_gain and digital_gain settle on reasonable values, then set exposure_mode
	# 	# to 'off'. (from doc)
	# 	self.camera.exposure_mode = 'off'

	# 	# To fix white balance, set the awb_mode to 'off', then set awb_gains to a (red, blue) tuple of gains.(from doc)
	# 	g = self.camera.awb_gains
	# 	self.camera.awb_mode = 'off'
	# 	self.camera.awb_gains = g

	# 	# Adjust focus to the best
	# 	self.adjust_lens(self.focal_length)

	# Close camera after use in next function

	def __init__(self):
		# vcm init
		self.last_value = None
		self.best_clarity = None
		self.poor_f_len_count = None
		self.best_index = None
		self.focal_length = None
		self.arducam_vcm.vcm_init()

		# open camera
		preview_config = self.picam2.create_still_configuration(main={"size": (640, 480)})
		self.picam2.configure(preview_config)

		self.picam2.set_controls({"AfMode": controls.AfModeEnum.Manual})
#		self.picam2.set_controls({"LensPosition": 5.6818181818})

		self.picam2.set_controls({"LensPosition": 10})

		# set camera resolution to 640x480(Small resolution for faster speeds.)
		# self.camera.resolution = (self.resolution[1], self.resolution[0])
		# time.sleep(0.1)
		# self.camera.shutter_speed = 30000

		# Determine focus values
		# Using this class in a Flask app runs init twice therefore self.camera.close() is needed
		# self.focus()

		# # Clear up
		# self.camera.close()

	def __del__(self):
		self.picam2.close()
