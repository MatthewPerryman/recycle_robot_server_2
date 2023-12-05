import cv2  # sudo apt-get install python-opencv
import numpy as py
import os
import time
import keyboard
from ctypes import *

# load arducam shared object file
arducam_vcm = CDLL('./lib/libarducam_vcm.so')
try:
	import picamera
	from picamera.array import PiRGBArray
except:
	sys.exit(0)


def adjust_lens(val):
	arducam_vcm.vcm_write(val)


# print("focus value: {}".format(val))

def sobel(img):
	img_gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
	img_sobel = cv2.Sobel(img_gray, cv2.CV_16U, 1, 1)
	return cv2.mean(img_sobel)[0]


def laplacian(img):
	img_gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
	img_sobel = cv2.Laplacian(img_gray, cv2.CV_16U)
	return cv2.mean(img_sobel)[0]


def calculation(camera):
	rawCapture = PiRGBArray(camera)
	camera.capture(rawCapture, format="bgr", use_video_port=True)
	image = rawCapture.array
	rawCapture.truncate(0)
	return laplacian(image)


def focus():
	print("Start focusing")

	max_index = 10
	max_value = 0.0
	last_value = 0.0
	dec_count = 0
	focal_distance = 10

	while True:
		# Adjust focus
		adjust_lens(focal_distance)
		# Take image and calculate image clarity
		val = calculation(camera)
		# Find the maximum image clarity
		if val > max_value:
			max_index = focal_distance
			max_value = val

		# If the image clarity starts to decrease
		if val < last_value:
			dec_count += 1
		else:
			dec_count = 0
		# Image clarity is reduced by six consecutive frames
		if dec_count > 6:
			break
		last_value = val

		# Increase the focal distance
		focal_distance += 15
		if focal_distance > 1000:
			break

	# Adjust focus to the best
	adjust_lens(max_index)
	return max_index, max_value


def take_photo(img_num):
	max_index, max_value = focus()

	time.sleep(1)
	# set camera resolution to 2592x1944
	# camera.resolution = (1920,1080)
	# save image to file.
	camera.capture("UnlabelledImages/img{}.jpg".format(img_num))
	print("max index = %d,max value = %lf" % (max_index, max_value))


if __name__ == "__main__":
	# vcm init
	arducam_vcm.vcm_init()
	# open camera
	camera = picamera.PiCamera()

	# open camera preview
	camera.start_preview()
	# set camera resolution to 640x480(Small resolution for faster speeds.)
	camera.resolution = (640, 480)
	time.sleep(0.1)
	camera.shutter_speed = 30000

	# https://picamera.readthedocs.io/en/release-1.13/recipes1.html?highlight=shutter%20speed#capturing-consistent-images
	# camera.iso = 100
	# camera.shutter_speed = camera.exposure_speed
	# camera.exposure_mode = 'off'

	# g = camera.awb_gains
	# camera.awb_mode = 'off'
	# camera.awb_gains = g

	img_count = int(input("Please enter next image number: "))

	while True:
		if keyboard.is_pressed('f'):  # if key 'f' is pressed
			print('Focussing')
			focus()
		elif keyboard.is_pressed('enter'):  # if key 'enter' is pressed
			print('Taking a Photo')
			take_photo(img_count)
			img_count += 1
		elif keyboard.is_pressed('esc'):  # if key 'esc' is pressed
			break

	camera.stop_preview()
	camera.close()
