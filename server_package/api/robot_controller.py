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

		# Check if the location is within bounds
		if self.swift.check_pos_is_limit(new_location) is False:
			# Wait = true to ensure response on move success or failure
			self.swift.set_position(x=new_location[0], y=new_location[1], z=new_location[2], wait=True, speed=speed)

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
			has_moved = True
		else:
			has_moved = False
		return has_moved

	## Move through every point in ONE continuous motion.
	##
	## The difference from calling move_to per point is the wait. Each point
	## is queued with wait=False, so the firmware's planner has the whole path
	## in hand and blends between segments instead of decelerating to a stop
	## at every one of them. A single flush at the end waits for the lot.
	##
	## Points are ABSOLUTE, in the same frame as move_to.
	def move_path(self, points, speed=100000):
		self.start_transmission()

		# Every point is bounds-checked BEFORE anything moves, so a bad point
		# late in the path cannot leave the arm stranded halfway along it.
		# check_pos_is_limit returns True when OUT of bounds - see move_to.
		for p in points:
			if self.swift.check_pos_is_limit(list(p)) is not False:
				print("move_path: out of bounds at ", p)
				self.end_transmission()
				return False

		print("Path: ", points)

		for p in points:
			self.swift.set_position(x=p[0], y=p[1], z=p[2],
			                        wait=False, speed=speed)

		# Waits for the whole queued path, not just the last command.
		self.swift.flush_cmd(wait_stop=True)

		# Wrist correction once, at the landing point. The intermediate points
		# are travel, and a wait=True wrist move between them would break the
		# blend that is the whole reason this method exists.
		uarm_angle = self.swift.get_servo_angle(0)
		angle_diff = uarm_angle - 90
		self.swift.set_wrist(90 + angle_diff, wait=True)

		self.end_transmission()
		return True

	## Is this position inside the arm's envelope? MOVES NOTHING.
	##
	## check_pos_is_limit returns True when the target is OUT of range, which
	## reads backwards - move_to tests it with "is False". This flips it so
	## callers get the obvious answer: True means reachable.
	##
	## Read-only, so the envelope can be mapped by probing rather than by
	## driving the arm at its own limits to find out where they are.
	def check_position(self, location):
		return self.swift.check_pos_is_limit(list(location)) is False

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

	## --- teach mode: stream position, record it on each button press ---
	##
	## Position is captured INSIDE the key callback from a continuously
	## streamed cache, so the recorded point is the one at the instant the
	## button was pressed. Calling get_position() synchronously from a
	## report callback would block the SDK's own receive thread.
	def start_teach_capture(self, interval=0.1):
		import threading
		self._teach_lock = threading.Lock()
		self._latest_pos = None
		self._key_events = []

		def on_pos(pos):
			try:
				self._latest_pos = [float(v) for v in pos[:3]]
			except Exception:
				pass

		def make_key_cb(button):
			def cb(status):
				import time
				with self._teach_lock:
					self._key_events.append({
						'button': button,
						'status': str(status),
						'position': self._latest_pos,
						't': time.time(),
					})
			return cb

		self.swift.set_report_position(interval)
		self.swift.register_report_position_callback(on_pos)
		self.swift.set_report_keys(True)
		self.swift.register_key0_callback(make_key_cb(0))
		self.swift.register_key1_callback(make_key_cb(1))
		return True

	def stop_teach_capture(self):
		try:
			self.swift.release_key0_callback()
			self.swift.release_key1_callback()
			self.swift.release_report_position_callback()
			self.swift.set_report_keys(False)
			self.swift.set_report_position(0)
		except Exception:
			pass
		return True

	## Return buffered button presses and clear the buffer.
	def drain_key_events(self):
		lock = getattr(self, '_teach_lock', None)
		if lock is None:
			return []
		with lock:
			events = list(self._key_events)
			self._key_events = []
		return events

	def latest_position(self):
		return getattr(self, '_latest_pos', None)

	## Mode decides where the TOOL CENTRE POINT sits (manual p.7):
	##   0 general/suction, 1 laser, 2 3D print, 3 pen/gripper
	## Our printed pointer is none of these, so the TCP sits wherever the
	## current mode puts it. That is why the TCP-to-tip offset is unknown
	## and has to be measured rather than read off the CAD.
	def device_info(self):
		info = {}
		try:
			info["mode"] = self.swift.get_mode()
		except Exception as e:
			info["mode"] = "error: %s" % e
		for k in ("device_type", "hardware_version", "firmware_version", "api_version"):
			try:
				info[k] = getattr(self.swift, k, None)
			except Exception:
				info[k] = None
		try:
			info["position"] = self.swift.get_position()
			info["servo_angles"] = [self.swift.get_servo_angle(i) for i in range(3)]
		except Exception as e:
			info["angles_err"] = str(e)
		return info

	## Release the servos so the arm can be moved by hand, or re-lock them.
	##
	## WARNING: detaching drops all holding torque. The arm WILL sag under
	## its own weight and whatever is mounted on it. Support it by hand
	## before enabling, and lower it somewhere safe first.
	def set_free_move(self, enable):
		self.start_transmission()
		if enable:
			res = self.swift.set_servo_detach()
		else:
			res = self.swift.set_servo_attach()
		self.end_transmission()
		return res

	## True if every servo reports attached (i.e. NOT in free-move).
	def is_attached(self):
		try:
			return [bool(self.swift.get_servo_attach(servo_id=i)) for i in range(3)]
		except Exception:
			return None

	## Flush queued commands and release the serial connection.
	## Safe to call more than once.
	def close(self):
		if getattr(self, "swift", None) is None:
			return
		try:
			self.swift.flush_cmd(wait_stop=True)
			self.swift.disconnect()
		finally:
			self.swift = None

	def __del__(self):
		self.close()
