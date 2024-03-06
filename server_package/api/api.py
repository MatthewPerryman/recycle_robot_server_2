# This code is only run on rpi server
import json
from flask import Flask, request, send_file, jsonify
from utils import logging
from .camera_controller import ImageStream
from .robot_controller import RobotController
import io
import numpy as np
import time

app = Flask(__name__)

# The camera is focussed here, therefore set up lighting before starting the app
image_stream = ImageStream()
controller = RobotController.RobotController()


# API Control of Robot Arm
@app.route('/update_position', methods=['POST'])
def increment_position():
	update_vector = request.content
	if (update_vector > -10) and (update_vector < 10):
		controller.move_by_increment(update_vector)
		return "API Check Success"
	else:
		return "API Check Fail"


@app.route('/move_by_vector/', methods=['POST'])
def move_by_vector():
	json_coord = json.loads(request.data)
	Xd = json_coord['Xd']
	Yd = json_coord['Yd']
	Zd = json_coord['Zd']

	response = controller.move_by_vector((Xd, Yd, Zd))

	return jsonify(response=response)


# Method to reset robot location
@app.route('/reset_robot', methods=['POST'])
def reset_robot():
	controller.reset(x=200, y=0, z=150)
	return 0


# Compact command get information for screw localising
@app.route('/get_images_for_depth', methods=['GET'])
def get_images_for_depth():
	Logging.write_log("server", "\nNew Run:\n")

	Logging.write_log("server", "Reset Location")
	reset_robot()

	Logging.write_log("server", "Call image_stream get depth images")
	# Take a photo, move the camera 1 cm to the right, take another
	img1, img2, f_len = image_stream.get_imgs_for_depth(controller.move_by_vector, Logging.write_log)
	print(f"img1: {img1.shape}, img2: {img2.shape}, f_len: {f_len}")

	Logging.write_log("server", "Compress Image")
	buffer = io.BytesIO()
	np.savez_compressed(buffer, img1, img2, f_len)
	buffer.seek(0)

	Logging.write_log("server", "Send Images")
	print(buffer)
	return send_file(buffer, as_attachment=True, download_name='depth_imgs.csv')

@app.route('/get_image_for_detection', methods=['GET'])
def get_image_for_detection():
	Logging.write_log("server", "\nNew Run:\n")

	Logging.write_log("server", "Reset Location")
	reset_robot()

	Logging.write_log("server", "Call image_stream get image of laptop")
	# Take a photo, move the camera 1 cm to the right, take another
	img1 = image_stream.take_photo()
	print(f"img1: {img1.shape}")

	Logging.write_log("server", "Compress Image")
	buffer = io.BytesIO()
	np.savez_compressed(buffer, img1)
	buffer.seek(0)

	Logging.write_log("server", "Send Images")
	print(buffer)
	return send_file(buffer, as_attachment=True, download_name='depth_imgs.csv')


# Set robot position
@app.route('/set_position/', methods=['POST'])
def set_position():
	new_json = json.loads(request.data)
	new_location = [new_json['Xd'], new_json['Yd'], new_json['Zd']]

	response = controller.move_to(new_location)

	return jsonify(response=response)


# Retrieve robot position
@app.route('/get_position/', methods=['GET'])
def get_position():
	location = controller.swift.get_position()
	return jsonify({"Xd": location[0], "Yd": location[1], "Zd": location[2]})

# Retrieve robot wrist angle
@app.route('/get_wrist_angle/', methods=['GET'])
def get_wrist_angle():
	angle = controller.swift.get_servo_angle(0)
	return jsonify({"angle": angle})


@app.route('/take_photo', methods=['GET'])
def take_photo():
	image_stream.set_focus_mode("Continuous")
	# wait for camera to settle
	time.sleep(2)

	image = image_stream.take_photo()
	Logging.write_log("server", "Compress Image")

	buffer = io.BytesIO()
	np.savez_compressed(buffer, image)
	buffer.seek(0)

	Logging.write_log("server", "Send Image")
	return send_file(buffer, as_attachment=True, download_name='depth_imgs.csv')

@app.route('/set_focus_mode', methods=['POST'])
def	set_focus_mode():
	json_data = json.loads(request.data)
	focus_mode = json_data['focus_mode']
	if focus_mode == "Continuous":
		image_stream.set_focus_mode("Continuous")
	elif focus_mode == "Manual":
		image_stream.set_focus_mode("Manual")

	return f"Focus Mode Set to {focus_mode}"


@app.route('/get_simple_photo', methods=['GET'])
def get_simple_photo():
	image = image_stream.take_simple_photo()
	Logging.write_log("server", "Compress Image")

	print(image.shape)
	buffer = io.BytesIO()
	np.savez_compressed(buffer, image)
	buffer.seek(0)

	Logging.write_log("server", "Send Image")
	return send_file(buffer, as_attachment=True, attachment_filename='singe_image.csv', mimetype="image/csv")


if __name__ == 'Server_Package.PiCode.rpiWebServer.API.api':
	try:
		app.run(port=1024, host='0.0.0.0')
	except KeyboardInterrupt:
		image_stream.__del__
		controller.__del__
		exit()
