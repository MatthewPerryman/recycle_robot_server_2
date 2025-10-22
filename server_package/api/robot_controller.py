import sys
sys.path.append('/home/ubuntu/uarm')
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
	def move_by_vector(self, vector, speed=100000):
		self.start_transmission()

		# Calculate new position and send update
		# TODO: Create vector class
		old = self.swift.get_position()

		print("Old Location: ", old)

		new_location = [old[0] + vector[0], old[1] + vector[1], old[2] + vector[2]]

		has_moved = self.move_to(new_location)

		self.end_transmission()

		return has_moved

	## Move the robot arm to this vector
	def move_to(self, new_location, speed=100000):
		self.start_transmission()

		print("New Location: ", new_location)
		print(self.swift.check_pos_is_limit(new_location))
		out_of_bounds = self.swift.check_pos_is_limit(new_location)

		# Check if the location is within bounds
		if out_of_bounds is False:
			# Wait = true to ensure response on move success or failure
			self.swift.set_position(x=new_location[0], y=new_location[1], z=new_location[2], wait=True, speed=speed)

			self.end_transmission()

			# TODO: Verify this actually moves the robot
			# #if actual_location[0] == new_location[0] and actual_location[1] == new_location[1] and actual_location[2] == new_location[2]:
			# # Calculate the angle of the robotic arm and move the wrist by that angle anticlockwise
			# uarm_angle = self.swift.get_servo_angle(0)
			# print(f"uarm_angle: {uarm_angle}")

			# # Calculate the difference subtracted from the original angle
			# angle_diff = uarm_angle-90
			# print(f"angle_diff: {angle_diff}")

			# # Move the wrist by the difference
			# self.swift.set_wrist(90+angle_diff, wait=True)
			has_moved = True
		else:
			has_moved = False
		return has_moved

	def stretch(self, length_mm, speed=100000):
		self.start_transmission()
		# Get current position
		current_pos = self.swift.get_position()
		print("Current Position: ", current_pos)

		# Calculate new position
		new_x = current_pos[0] + length_mm
		new_location = [new_x, current_pos[1], current_pos[2]]

		print("New Location: ", new_location)
		
		out_of_bounds = self.swift.check_pos_is_limit(new_location)
		if out_of_bounds is True:
			print("Stretch out of bounds, aborting")
			self.end_transmission()
			return False
		else:
			print("New Position: ", new_location)

			# Move to new position
			has_moved = self.move_to(new_location)

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
