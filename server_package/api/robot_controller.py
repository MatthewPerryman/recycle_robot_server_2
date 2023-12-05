from uarm.wrapper import SwiftAPI


class RobotController:
	swift = None

	def start_transmission(self):
		self.swift.flush_cmd(wait_stop=True)
		self.swift.waiting_ready(timeout=3)

	## Clear cmd buffer and close connection
	def end_transmission(self):
		self.swift.flush_cmd(wait_stop=True)
		self.swift.waiting_ready(timeout=3)

	## Move the robot arm by an increment value
	def move_by_vector(self, vector, reverse_vector=False, speed=100000):
		self.start_transmission()

		# Calculate new position and send update
		# TODO: Create vector class
		old = self.swift.get_position()

		print("Old Location: ", old)

		if reverse_vector:
			new_location = [old[0] - vector[0], old[1] - vector[1], old[2] - vector[2]]
		else:
			new_location = [old[0] + vector[0], old[1] + vector[1], old[2] + vector[2]]

		response = self.move_to(new_location)

		self.end_transmission()

		return response

	## Move the robot arm to this vector
	def move_to(self, new_location, speed=100000):
		self.start_transmission()

		print("New Location: ", new_location)
		print(self.swift.check_pos_is_limit(new_location))

		# Check if the location is within bounds
		if self.swift.check_pos_is_limit(new_location) is False:
			# Wait = true to ensure response on move success or failure
			response = self.swift.set_position(x=new_location[0], y=new_location[1], z=new_location[2], wait=True, speed=speed)

			self.end_transmission()

			#if actual_location[0] == new_location[0] and actual_location[1] == new_location[1] and actual_location[2] == new_location[2]:
			# Calculate the angle of the robotic arm and move the wrist by that angle anticlockwise
			uarm_angle = self.swift.get_servo_angle(0)
			print(f"uarm_angle: {uarm_angle}")

			# Calculate the difference subtracted from the original angle
			angle_diff = uarm_angle-90
			print(f"angle_diff: {angle_diff}")

			# Move the wrist by the difference
			self.swift.set_wrist(90+angle_diff, wait=True)
			response = True
		else:
			response = False
		return response

	# Reset robot location
	def reset(self, x=200, y=0, z=150):
		self.start_transmission()
		# Wait = true to ensure response on move success or failure
		self.swift.reset(speed=100000, x=x, y=y, z=z)
		self.end_transmission()
		print(f"Resetting to {x}, {y}, {z}")

	## Create the API context and put robot on standby
	def __init__(self):
		if self.swift == None:
			self.swift = SwiftAPI()
			self.swift.waiting_ready(timeout=3)

		self.swift.flush_cmd(wait_stop=True)

		# Reset the arms location
		self.swift.reset(x=200, y=0, z=150)
		self.swift.waiting_ready(timeout=3)

		self.end_transmission()

	def __del__(self):
		self.swift.flush_cmd(wait_stop=True)
		self.swift.disconnect()
