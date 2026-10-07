# This code is only run on rpi server
import json
from flask import Flask, request, send_file, jsonify
from utils import logging
from .camera_controller import ImageStream
from .robot_controller import RobotController
from .screw_driver_controller import (ScrewDriverConfig, ScrewDriverController,
                                      ScrewDriverError)
import io
import os
import numpy as np
import time

app = Flask(__name__)

# The camera is focussed here, therefore set up lighting before starting the app
controller = RobotController()
image_stream = ImageStream(controller)  # Pass the shared controller instance

# The screwdriver is optional: if its board is unplugged or its config is
# wrong, the server still starts and its endpoints say why. POST
# /screw_driver/connect/ after fixing it - no restart, so the arm stays put.
SCREW_DRIVER_CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                        "..", "..", "screw_driver_config.json")
screw_driver = None
screw_driver_problem = "not connected yet"


def connect_screw_driver():
	global screw_driver, screw_driver_problem
	if screw_driver is not None:
		screw_driver.close()
		screw_driver = None
	try:
		config = ScrewDriverConfig.from_json_file(SCREW_DRIVER_CONFIG_PATH)
		screw_driver = ScrewDriverController(config)
		screw_driver_problem = None
	except (OSError, ValueError, TypeError, ScrewDriverError) as exc:
		screw_driver_problem = "%s: %s" % (type(exc).__name__, exc)
	logging.write_log("server", "screw driver: %s" % (screw_driver_problem or "connected"))
	return screw_driver_problem


connect_screw_driver()

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

	has_moved = controller.move_by_vector((Xd, Yd, Zd))

	return jsonify(response=has_moved)


# Move through a list of ABSOLUTE positions as one continuous motion, without
# stopping at the intermediate ones. POST {"points": [{"Xd":..,"Yd":..,"Zd":..}]}
@app.route('/move_path/', methods=['POST'])
def move_path():
	points = json.loads(request.data)['points']

	response = controller.move_path(
		[[p['Xd'], p['Yd'], p['Zd']] for p in points])

	return jsonify(response=response)


# Is a position reachable? Returns {"reachable": bool} and MOVES NOTHING, so
# the envelope can be mapped safely. POST {"Xd":..,"Yd":..,"Zd":..}
@app.route('/check_position/', methods=['POST'])
def check_position():
	json_coord = json.loads(request.data)
	location = [json_coord['Xd'], json_coord['Yd'], json_coord['Zd']]

	return jsonify(reachable=controller.check_position(location))


# Method to reset robot location
@app.route('/reset_robot', methods=['POST'])
def reset_robot():
	controller.reset(x=200, y=0, z=150)
	return 0


# Compact command get information for screw localising
@app.route('/get_images_for_depth', methods=['GET'])
def get_images_for_depth():
	logging.write_log("server", "\nNew Run:\n")

	logging.write_log("server", "Reset Location")
	reset_robot()

	logging.write_log("server", "Call image_stream get depth images")
	# Take a photo, move the camera 1 cm to the right, take another
	img1, img2 = image_stream.get_imgs_for_depth(logging.write_log)
	print(f"img1: {img1.shape}, img2: {img2.shape}, f_len: {f_len}")

	logging.write_log("server", "Compress Image")
	buffer = io.BytesIO()
	np.savez_compressed(buffer, img1, img2)
	buffer.seek(0)

	logging.write_log("server", "Send Images")
	print(buffer)
	return send_file(buffer, as_attachment=True, download_name='depth_imgs.csv')

@app.route('/get_image_for_detection', methods=['GET'])
def get_image_for_detection():
	logging.write_log("server", "\nNew Run:\n")

	logging.write_log("server", "Reset Location")
	reset_robot()

	logging.write_log("server", "Call image_stream get image of laptop")
	# Take a photo, move the camera 1 cm to the right, take another
	img1 = image_stream.take_photo()
	print(f"img1: {img1.shape}")

	logging.write_log("server", "Compress Image")
	buffer = io.BytesIO()
	np.savez_compressed(buffer, img1)
	buffer.seek(0)

	logging.write_log("server", "Send Images")
	print(buffer)
	return send_file(buffer, as_attachment=True, download_name='depth_imgs.csv')


# Set robot position
@app.route('/set_position/', methods=['POST'])
def set_position():
	new_json = json.loads(request.data)
	new_location = [new_json['Xd'], new_json['Yd'], new_json['Zd']]

	response = controller.move_to(new_location)

	return jsonify(response=response)


# Start/stop streaming position and reporting base-button presses.
# POST {"enable": true} before a teach session.
@app.route('/teach_capture/', methods=['POST'])
def teach_capture():
	enable = bool(json.loads(request.data).get('enable', False))
	if enable:
		controller.start_teach_capture()
	else:
		controller.stop_teach_capture()
	logging.write_log("server", "teach_capture enable=%s" % enable)
	return jsonify(teach_capture=enable)


# Drain buffered button presses. Each carries the arm position AT THE
# MOMENT the button was pressed, not when this was polled.
# status '1' = short press, '2' = long press. button 0 or 1.
@app.route('/key_events/', methods=['GET'])
def key_events():
	return jsonify(events=controller.drain_key_events(),
	               latest_position=controller.latest_position())

# Release or re-lock the servos so the arm can be positioned by hand.
# POST {"enable": true} to free it, {"enable": false} to lock it.
#
# WARNING: freeing the arm removes ALL holding torque - it will sag under
# the weight of the mount and camera. Support it before calling this.
@app.route('/free_move/', methods=['POST'])
def free_move():
	enable = bool(json.loads(request.data).get('enable', False))
	result = controller.set_free_move(enable)
	logging.write_log("server", "free_move enable=%s -> %s" % (enable, result))
	return jsonify(free_move=enable, response=str(result))


# Which servos are currently attached (locked)?
@app.route('/servo_state/', methods=['GET'])
def servo_state():
	return jsonify(attached=controller.is_attached())

# Arm mode, versions and joint angles. Mode decides where the TCP is.
@app.route('/device_info/', methods=['GET'])
def device_info():
	return jsonify(controller.device_info())

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
	logging.write_log("server", "Compress Image")

	buffer = io.BytesIO()
	np.savez_compressed(buffer, image)
	buffer.seek(0)

	logging.write_log("server", "Send Image")
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
	image = image_stream.take_photo()
	logging.write_log("server", "Compress Image")

	print(image.shape)
	buffer = io.BytesIO()
	np.savez_compressed(buffer, image)
	buffer.seek(0)

	logging.write_log("server", "Send Image")
	return send_file(buffer, as_attachment=True, attachment_filename='singe_image.csv', mimetype="image/csv")


# --- Screwdriver ----------------------------------------------------------
# Directions are as seen looking down the bit at the screw head: clockwise
# tightens, anticlockwise loosens. See screw_driver_controller.py.

def screw_driver_unavailable():
	return jsonify(error="screw driver not connected: %s" % screw_driver_problem), 503


# The board stopped answering mid-command (cable out, power off): say so
# clearly instead of a bare 500. /screw_driver/connect/ once it is back.
@app.errorhandler(ScrewDriverError)
def screw_driver_lost(exc):
	return jsonify(error="screw driver stopped answering: %s" % exc), 503


@app.route('/screw_driver/connect/', methods=['POST'])
def screw_driver_connect():
	problem = connect_screw_driver()
	return jsonify(connected=problem is None, problem=problem)


@app.route('/screw_driver/status/', methods=['GET'])
def screw_driver_status():
	if screw_driver is None:
		return screw_driver_unavailable()
	return jsonify(screw_driver.status())


# POST {"degrees": -720, "speed_rpm": 10}  (speed_rpm optional)
# Positive = clockwise. Waits until the rotation ends, then returns what
# happened, including the load-against-angle samples.
@app.route('/screw_driver/rotate_by_degrees/', methods=['POST'])
def screw_driver_rotate_by_degrees():
	if screw_driver is None:
		return screw_driver_unavailable()
	request_body = json.loads(request.data)
	try:
		result = screw_driver.rotate_by_degrees(float(request_body['degrees']),
		                                        request_body.get('speed_rpm'))
	except ValueError as exc:
		return jsonify(error=str(exc)), 400
	return jsonify(result)


# POST {"direction": "anticlockwise", "speed_rpm": 10, "max_seconds": 5}
# Starts turning and returns straight away. It stops on /screw_driver/stop/,
# at the load limit, or after max_seconds - whichever comes first.
@app.route('/screw_driver/rotate/', methods=['POST'])
def screw_driver_rotate():
	if screw_driver is None:
		return screw_driver_unavailable()
	request_body = json.loads(request.data)
	try:
		result = screw_driver.rotate(request_body['direction'],
		                             request_body.get('speed_rpm'),
		                             request_body.get('max_seconds'))
	except ValueError as exc:
		return jsonify(error=str(exc)), 400
	return jsonify(result)


@app.route('/screw_driver/stop/', methods=['POST'])
def screw_driver_stop():
	if screw_driver is None:
		return screw_driver_unavailable()
	return jsonify(last_rotation=screw_driver.stop())
