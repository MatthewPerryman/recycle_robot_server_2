"""Tests for the screwdriver controller, against a simulated servo.

	python tests/test_screw_driver_controller.py

Runs anywhere with feetech-servo-sdk installed - no Pi, camera, arm or servo.
The controller module is loaded straight from its file, because importing
server_package runs api.py, which needs the Pi's camera.
"""
import importlib.util
import os
import threading
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
MODULE_PATH = os.path.join(HERE, "..", "server_package", "api", "screw_driver_controller.py")
spec = importlib.util.spec_from_file_location("screw_driver_controller", MODULE_PATH)
sdc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sdc)


class FakeServo:
	"""Answers reads and writes like the STS3215, turning at whatever speed
	was last written, in real time. load_for_degrees(degrees turned) -> load %."""

	def __init__(self, start_position=0, mode=sdc.WHEEL_MODE, load_for_degrees=None,
	             answers=True):
		self.memory = {sdc.MODE: mode, sdc.GOAL_SPEED: 0, sdc.PRESENT_VOLTAGE: 75,
		               sdc.PRESENT_TEMPERATURE: 30, sdc.PRESENT_CURRENT: 12}
		self.position_steps = float(start_position)   # unwrapped
		self.start_position = float(start_position)
		self.load_for_degrees = load_for_degrees or (lambda degrees: 5.0)
		self.answers = answers
		self.writes = []                             # (address, value) in order
		self.speeds_written = []
		self._last_update = time.monotonic()
		self._lock = threading.Lock()

	def _speed(self):
		return sdc.decode_sign(self.memory[sdc.GOAL_SPEED], sdc.SPEED_SIGN_BIT)

	def _advance(self):
		now = time.monotonic()
		self.position_steps += self._speed() * (now - self._last_update)
		self._last_update = now

	def read(self, address):
		with self._lock:
			self._advance()
			if address == sdc.PRESENT_POSITION:
				return int(self.position_steps) % sdc.STEPS_PER_TURN
			if address == sdc.PRESENT_SPEED:
				return sdc.encode_sign(self._speed(), sdc.SPEED_SIGN_BIT)
			if address == sdc.PRESENT_LOAD:
				degrees = abs(self.position_steps - self.start_position) * sdc.DEGREES_PER_STEP
				tenths = round(self.load_for_degrees(degrees) * 10)
				return sdc.encode_sign(tenths, sdc.LOAD_SIGN_BIT)
			return self.memory.get(address, 0)

	def write(self, address, value):
		with self._lock:
			self._advance()
			self.memory[address] = value
			self.writes.append((address, value))
			if address == sdc.GOAL_SPEED:
				self.speeds_written.append(sdc.decode_sign(value, sdc.SPEED_SIGN_BIT))


class FakePort:
	def __init__(self, name):
		self.name = name
		self.closed = False

	def openPort(self):
		return True

	def setBaudRate(self, baud):
		return True

	def closePort(self):
		self.closed = True


class FakePacketHandler:
	"""The four calls the controller uses, in the SDK's (value, comm, err) shapes."""

	def __init__(self, servo):
		self.servo = servo

	def _reply(self):
		return sdc.COMM_SUCCESS if self.servo.answers else -1001

	def ping(self, port, servo_id):
		return 777, self._reply(), 0

	def read1ByteTxRx(self, port, servo_id, address):
		return self.servo.read(address), self._reply(), 0

	def read2ByteTxRx(self, port, servo_id, address):
		return self.servo.read(address), self._reply(), 0

	def write1ByteTxRx(self, port, servo_id, address, value):
		self.servo.write(address, value)
		return self._reply(), 0

	def write2ByteTxRx(self, port, servo_id, address, value):
		self.servo.write(address, value)
		return self._reply(), 0

	def getTxRxResult(self, comm):
		return "no status packet (simulated)"

	def getRxPacketError(self, err):
		return "simulated error"


def make_controller(servo, **config_overrides):
	sdc.PortHandler = FakePort
	sdc.PacketHandler = lambda protocol_end: FakePacketHandler(servo)
	settings = dict(port="FAKE", default_speed_rpm=40.0, max_speed_rpm=40.0,
	                max_rotation_seconds=5.0)
	settings.update(config_overrides)
	return sdc.ScrewDriverController(sdc.ScrewDriverConfig(**settings))


class StepsMovedTest(unittest.TestCase):
	def test_wraps_both_ways(self):
		self.assertEqual(sdc.steps_moved(100, 150), 50)
		self.assertEqual(sdc.steps_moved(4090, 10), 16)
		self.assertEqual(sdc.steps_moved(10, 4090), -16)
		self.assertEqual(sdc.steps_moved(150, 100), -50)

	def test_sign_encoding_round_trips(self):
		for value in (0, 1, 500, -500, 1023):
			self.assertEqual(sdc.decode_sign(sdc.encode_sign(value, 15), 15), value)


class ConfigTest(unittest.TestCase):
	def test_extra_load_limit_must_be_below_torque_limit(self):
		with self.assertRaises(ValueError):
			sdc.ScrewDriverConfig(port="X", torque_limit_percent=30, extra_load_limit_percent=30)

	def test_defaults_are_valid(self):
		sdc.ScrewDriverConfig(port="X")


class RotateByDegreesTest(unittest.TestCase):
	def test_clockwise_turns_the_angle_and_stops(self):
		servo = FakeServo()
		result = make_controller(servo).rotate_by_degrees(90)
		self.assertEqual(result["stopped_because"], "reached the angle")
		self.assertEqual(result["direction"], "clockwise")
		self.assertGreaterEqual(result["degrees_turned"], 90)
		self.assertLess(result["degrees_turned"], 110)
		self.assertGreater(servo.speeds_written[0], 0)
		self.assertEqual(servo.speeds_written[-1], 0)
		self.assertTrue(result["samples"])

	def test_anticlockwise_drives_the_servo_backwards(self):
		servo = FakeServo()
		result = make_controller(servo).rotate_by_degrees(-90)
		self.assertEqual(result["direction"], "anticlockwise")
		self.assertEqual(result["requested_degrees"], -90)
		self.assertGreaterEqual(result["degrees_turned"], 90)
		self.assertLess(servo.speeds_written[0], 0)

	def test_config_flips_which_way_is_clockwise(self):
		servo = FakeServo()
		make_controller(servo, clockwise_is_positive_speed=False).rotate_by_degrees(90)
		self.assertLess(servo.speeds_written[0], 0)

	def test_counts_across_the_wrap(self):
		servo = FakeServo(start_position=4000)
		result = make_controller(servo).rotate_by_degrees(360)
		self.assertGreaterEqual(result["degrees_turned"], 360)
		self.assertLess(result["degrees_turned"], 380)

	# make_controller turns at 40 rpm by default: free spin 0.85 x 40 = 34%.

	def test_stops_at_the_extra_load_limit(self):
		servo = FakeServo(load_for_degrees=lambda degrees: 34.0 + 22.0 if degrees > 30 else 34.0)
		result = make_controller(servo, torque_limit_percent=60).rotate_by_degrees(720)
		self.assertEqual(result["stopped_because"], "load limit")
		self.assertLess(result["degrees_turned"], 60)
		self.assertEqual(result["free_spin_load_percent"], 34.0)

	def test_free_spin_load_alone_does_not_stop_it(self):
		# 30 rpm read 25.6% in the air on 2026-10-07 and tripped the old fixed
		# 25% limit; above free spin it is nothing.
		servo = FakeServo(load_for_degrees=lambda degrees: 25.6)
		result = make_controller(servo).rotate_by_degrees(90, speed_rpm=30)
		self.assertEqual(result["stopped_because"], "reached the angle")

	def test_stops_early_at_a_given_extra_load_threshold(self):
		servo = FakeServo(load_for_degrees=lambda degrees: 34.0 + 10.0 if degrees > 30 else 34.0)
		result = make_controller(servo).rotate_by_degrees(-720, stop_above_extra_load_percent=8)
		self.assertEqual(result["stopped_because"], "load threshold")
		self.assertLess(result["degrees_turned"], 60)
		self.assertEqual(servo.speeds_written[-1], 0)

	def test_extra_load_threshold_must_be_below_the_extra_load_limit(self):
		controller = make_controller(FakeServo())
		for threshold in (0, 20, 40):
			with self.assertRaises(ValueError):
				controller.rotate_by_degrees(90, stop_above_extra_load_percent=threshold)

	def test_refuses_a_speed_whose_free_spin_leaves_no_room_under_the_torque_limit(self):
		# 0.85 x 40 + 20 = 54, not below a 50% torque limit: it could stall unseen.
		controller = make_controller(FakeServo(), torque_limit_percent=50)
		with self.assertRaises(ValueError):
			controller.rotate_by_degrees(90, speed_rpm=40)
		controller.rotate_by_degrees(90, speed_rpm=20)       # 37: fits

	def test_times_out(self):
		servo = FakeServo()
		result = make_controller(servo, max_rotation_seconds=0.3).rotate_by_degrees(36000)
		self.assertEqual(result["stopped_because"], "timed out")
		self.assertEqual(servo.speeds_written[-1], 0)

	def test_stop_from_another_thread_is_not_missed(self):
		servo = FakeServo()
		controller = make_controller(servo)
		threading.Timer(0.2, controller.stop).start()
		result = controller.rotate_by_degrees(36000)
		self.assertEqual(result["stopped_because"], "stop requested")
		self.assertFalse(controller.is_rotating())

	def test_mode_only_written_when_it_changes(self):
		servo = FakeServo(mode=sdc.WHEEL_MODE)
		make_controller(servo).rotate_by_degrees(45)
		self.assertNotIn(sdc.MODE, [address for address, _ in servo.writes])
		servo = FakeServo(mode=0)
		controller = make_controller(servo)
		controller.rotate_by_degrees(45)
		controller.rotate_by_degrees(45)
		self.assertEqual([address for address, _ in servo.writes].count(sdc.MODE), 1)

	def test_speed_above_the_maximum_is_refused(self):
		with self.assertRaises(ValueError):
			make_controller(FakeServo()).rotate_by_degrees(90, speed_rpm=100)


class ContinuousRotateTest(unittest.TestCase):
	def test_rotate_returns_straight_away_and_stop_stops_it(self):
		servo = FakeServo()
		controller = make_controller(servo)
		started = time.monotonic()
		controller.rotate("anticlockwise")
		self.assertLess(time.monotonic() - started, 0.5)
		time.sleep(0.2)
		self.assertTrue(controller.is_rotating())
		result = controller.stop()
		self.assertFalse(controller.is_rotating())
		self.assertEqual(result["stopped_because"], "stop requested")
		self.assertEqual(servo.speeds_written[-1], 0)

	def test_rotate_stops_itself_after_max_seconds(self):
		servo = FakeServo()
		controller = make_controller(servo)
		controller.rotate("clockwise", max_seconds=0.2)
		time.sleep(0.8)
		self.assertFalse(controller.is_rotating())
		self.assertEqual(controller.last_rotation["stopped_because"], "timed out")
		self.assertEqual(servo.speeds_written[-1], 0)

	def test_rejects_unknown_direction(self):
		with self.assertRaises(ValueError):
			make_controller(FakeServo()).rotate("sideways")


class ConnectionTest(unittest.TestCase):
	def test_no_reply_raises_and_closes_the_port(self):
		with self.assertRaises(sdc.ScrewDriverError):
			make_controller(FakeServo(answers=False))

	def test_status_reports_the_readings(self):
		status = make_controller(FakeServo()).status()
		self.assertEqual(status["voltage"], 7.5)
		self.assertEqual(status["temperature_c"], 30)
		self.assertFalse(status["rotating"])


if __name__ == "__main__":
	unittest.main()
