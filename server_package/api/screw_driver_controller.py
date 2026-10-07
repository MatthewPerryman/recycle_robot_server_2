"""The screwdriver: a Feetech STS3215 serial-bus servo, driven through a
Waveshare bus-servo board plugged into one of the Pi's USB ports.

The servo does nothing on power alone. It waits for commands on its serial
bus, and everything here is a read or a write to a numbered address in the
servo's own memory - the two tables below.

DIRECTIONS

Clockwise and anticlockwise are as seen looking down the bit at the screw
head, the way screws are described: clockwise tightens, anticlockwise
loosens. Which way the servo's own positive speed turns the bit depends on
how it is mounted, so the config says (clockwise_is_positive_speed). Check
it once by eye with a small clockwise rotation.

THE LOAD TRACE

rotate_by_degrees returns, as well as how far it turned, a sample every
~10 ms of the angle turned and the load. That is the measurement for
"is the screw's resistance rising, levelling off or falling": a seized
screw climbs to the load limit, one that has broken free drops to a low
plateau. The load is the servo's own reading, in % of its maximum torque.

SAFETY

Every rotation - including rotate(), which returns straight away - runs
the same loop, which stops the motor if the load passes load_limit_percent,
if it has run longer than max_rotation_seconds, or if stop() is called. A
laptop that loses its connection mid-rotation cannot leave it spinning.
"""
import json
import threading
import time
from dataclasses import dataclass

from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

PROTOCOL_END = 0                # byte order: 0 for STS/SMS servos, 1 for SCS
STEPS_PER_TURN = 4096
HALF_TURN = STEPS_PER_TURN // 2
DEGREES_PER_STEP = 360 / STEPS_PER_TURN

# Readings (the servo keeps these up to date)
PRESENT_POSITION    = 56        # 2 bytes, 0-4095 for one turn
PRESENT_SPEED       = 58        # 2 bytes, steps per second, sign in bit 15
PRESENT_LOAD        = 60        # 2 bytes, tenths of a % of max torque, sign in bit 10
PRESENT_VOLTAGE     = 62        # 1 byte, tenths of a volt
PRESENT_TEMPERATURE = 63        # 1 byte, degrees C
PRESENT_CURRENT     = 69        # 2 bytes, ~6.5 mA per unit - address and scale UNVERIFIED

# Settings (written by us)
MODE          = 33              # 1 byte: 0 = position, 1 = wheel (continuous)
TORQUE_ENABLE = 40              # 1 byte: 1 = holds and drives, 0 = goes limp
ACCELERATION  = 41              # 1 byte: 0-254, in units of 100 steps/s per second
GOAL_SPEED    = 46              # 2 bytes, steps per second, sign in bit 15 (wheel mode)
TORQUE_LIMIT  = 48              # 2 bytes, tenths of a % of max torque (1000 = 100%)

WHEEL_MODE = 1
SPEED_SIGN_BIT = 15
LOAD_SIGN_BIT = 10

SAMPLE_INTERVAL_SECONDS = 0.01  # time between readings while rotating
COAST_SETTLE_SECONDS = 0.15     # after "stop", let it decelerate before the last reading


class ScrewDriverError(Exception):
	"""The servo could not be reached, or the port could not be opened."""


@dataclass
class ScrewDriverConfig:
	port: str                               # Pi: a /dev/serial/by-id/... path; laptop: e.g. "COM4"
	servo_id: int = 1
	baud_rate: int = 1_000_000
	clockwise_is_positive_speed: bool = True    # flip if "clockwise" turns the bit anticlockwise
	default_speed_rpm: float = 10.0
	max_speed_rpm: float = 40.0
	acceleration: int = 50                  # 0-254, units of 100 steps/s per second
	torque_limit_percent: float = 30.0      # the most torque the servo may use
	load_limit_percent: float = 25.0        # stop a rotation above this load
	max_rotation_seconds: float = 15.0      # stop any rotation that runs longer than this

	def __post_init__(self):
		if not 0 < self.torque_limit_percent <= 100:
			raise ValueError("torque_limit_percent must be between 0 and 100")
		# The load reading cannot go above the torque limit, so a load limit at
		# or above it would never trigger - the servo would just sit stalled.
		if not 0 < self.load_limit_percent < self.torque_limit_percent:
			raise ValueError("load_limit_percent (%s) must be above 0 and below "
			                 "torque_limit_percent (%s)"
			                 % (self.load_limit_percent, self.torque_limit_percent))
		if not 0 < self.default_speed_rpm <= self.max_speed_rpm:
			raise ValueError("default_speed_rpm must be above 0 and no more than max_speed_rpm")
		if not 0 <= self.acceleration <= 254:
			raise ValueError("acceleration must be 0-254")
		if self.max_rotation_seconds <= 0:
			raise ValueError("max_rotation_seconds must be above 0")

	@classmethod
	def from_json_file(cls, path):
		with open(path) as config_file:
			return cls(**json.load(config_file))


def steps_moved(previous, current):
	"""Steps moved between two position readings, allowing for the wrap from
	4095 back to 0. Readings are always 0-4095, so the plain difference is
	never more than one turn out and one correction is enough."""
	difference = current - previous
	if difference > HALF_TURN:
		difference -= STEPS_PER_TURN
	elif difference < -HALF_TURN:
		difference += STEPS_PER_TURN
	return difference


def decode_sign(raw, sign_bit):
	"""The servo stores a negative number as its size plus one bit for the sign."""
	size = raw & ((1 << sign_bit) - 1)      # keep only the bits below the sign bit
	is_negative = raw & (1 << sign_bit)
	return -size if is_negative else size


def encode_sign(value, sign_bit):
	return (-value) | (1 << sign_bit) if value < 0 else value


def rpm_to_steps_per_second(rpm):
	return round(rpm * STEPS_PER_TURN / 60)


class ScrewDriverController:
	def __init__(self, config):
		self.config = config
		self.last_rotation = None           # result of the most recent rotation
		self.last_servo_error = None        # the servo's last complaint, if any
		self._bus_lock = threading.Lock()   # one message on the serial bus at a time
		self._stop_requested = threading.Event()
		self._rotation_finished = threading.Event()   # set whenever nothing is rotating
		self._rotation_finished.set()
		self._mode = None
		self._connected = False

		self._port = PortHandler(config.port)
		self._packet_handler = PacketHandler(PROTOCOL_END)
		try:
			if not self._port.openPort():
				raise ScrewDriverError("could not open %s" % config.port)
			if not self._port.setBaudRate(config.baud_rate):
				raise ScrewDriverError("could not set the baud rate to %s" % config.baud_rate)
			with self._bus_lock:
				model_number, comm, err = self._packet_handler.ping(self._port, config.servo_id)
			self._check(comm, err, "ping")
			self.model_number = model_number
			self._mode = self._read_1_byte(MODE, "mode")
			self._connected = True
		except Exception:
			self.close()
			raise

	## --- talking to the servo -------------------------------------------

	def _check(self, comm, err, what):
		if comm != COMM_SUCCESS:
			raise ScrewDriverError("no reply to %s: %s"
			                       % (what, self._packet_handler.getTxRxResult(comm)))
		if err:
			self.last_servo_error = "%s: %s" % (what, self._packet_handler.getRxPacketError(err))
			print("servo flagged an error during", self.last_servo_error)

	def _read_1_byte(self, address, what):
		with self._bus_lock:
			value, comm, err = self._packet_handler.read1ByteTxRx(
				self._port, self.config.servo_id, address)
		self._check(comm, err, what)
		return value

	def _read_2_bytes(self, address, what):
		with self._bus_lock:
			value, comm, err = self._packet_handler.read2ByteTxRx(
				self._port, self.config.servo_id, address)
		self._check(comm, err, what)
		return value

	def _write_1_byte(self, address, value, what):
		with self._bus_lock:
			comm, err = self._packet_handler.write1ByteTxRx(
				self._port, self.config.servo_id, address, value)
		self._check(comm, err, what)

	def _write_2_bytes(self, address, value, what):
		with self._bus_lock:
			comm, err = self._packet_handler.write2ByteTxRx(
				self._port, self.config.servo_id, address, value)
		self._check(comm, err, what)

	def _read_load_percent(self):
		return decode_sign(self._read_2_bytes(PRESENT_LOAD, "load"), LOAD_SIGN_BIT) / 10

	## --- rotating -------------------------------------------------------

	def _servo_direction(self, clockwise):
		"""+1 or -1: the sign of the servo's own speed that turns the bit this way."""
		positive_is_clockwise = self.config.clockwise_is_positive_speed
		return 1 if clockwise == positive_is_clockwise else -1

	def _prepare_wheel_mode(self):
		self._write_2_bytes(TORQUE_LIMIT, round(self.config.torque_limit_percent * 10), "torque limit")
		self._write_1_byte(ACCELERATION, self.config.acceleration, "acceleration")
		# The mode is a stored setting; only write it when it needs to change,
		# because that memory wears out after many writes.
		if self._mode != WHEEL_MODE:
			self._write_1_byte(MODE, WHEEL_MODE, "mode")
			self._mode = WHEEL_MODE
		self._write_1_byte(TORQUE_ENABLE, 1, "torque enable")

	def _speed_rpm_or_default(self, speed_rpm):
		speed_rpm = self.config.default_speed_rpm if speed_rpm is None else float(speed_rpm)
		if not 0 < speed_rpm <= self.config.max_speed_rpm:
			raise ValueError("speed_rpm must be above 0 and no more than %s"
			                 % self.config.max_speed_rpm)
		return speed_rpm

	def _stop_above_load_or_none(self, stop_above_load_percent):
		"""A lower load threshold for one rotation - e.g. "stop once the bit has
		bitten" - checked as well as the config's load_limit_percent, never
		instead of it, so it has to be below that limit to mean anything."""
		if stop_above_load_percent is None:
			return None
		stop_above_load_percent = float(stop_above_load_percent)
		if not 0 < stop_above_load_percent < self.config.load_limit_percent:
			raise ValueError("stop_above_load_percent must be above 0 and below "
			                 "load_limit_percent (%s)" % self.config.load_limit_percent)
		return stop_above_load_percent

	def _run_rotation(self, clockwise, speed_rpm, target_degrees, max_seconds,
	                  stop_above_load_percent=None):
		"""The one rotation loop. Turns until target_degrees (None = until
		stopped), the load limit, stop_above_load_percent (if given),
		max_seconds, or stop(). Returns what happened."""
		servo_direction = self._servo_direction(clockwise)
		self._prepare_wheel_mode()
		previous_position = self._read_2_bytes(PRESENT_POSITION, "start position")
		servo_steps = 0                     # signed, in the servo's own direction;
		                                    # x servo_direction = progress the requested way
		samples = []                        # [seconds, degrees turned, load %]
		peak_load_percent = 0.0
		stopped_because = "reached the angle"
		started = time.monotonic()
		try:
			speed = rpm_to_steps_per_second(speed_rpm) * servo_direction
			self._write_2_bytes(GOAL_SPEED, encode_sign(speed, SPEED_SIGN_BIT), "speed")
			while True:
				if self._stop_requested.is_set():
					stopped_because = "stop requested"
					break
				current_position = self._read_2_bytes(PRESENT_POSITION, "position")
				servo_steps += steps_moved(previous_position, current_position)
				previous_position = current_position
				degrees_turned = servo_steps * servo_direction * DEGREES_PER_STEP
				load_percent = self._read_load_percent()
				elapsed_seconds = time.monotonic() - started
				samples.append([round(elapsed_seconds, 3), round(degrees_turned, 1), load_percent])
				peak_load_percent = max(peak_load_percent, abs(load_percent))
				if target_degrees is not None and degrees_turned >= target_degrees:
					break
				if abs(load_percent) > self.config.load_limit_percent:
					stopped_because = "load limit"
					break
				if stop_above_load_percent is not None and abs(load_percent) > stop_above_load_percent:
					stopped_because = "load threshold"
					break
				if elapsed_seconds > max_seconds:
					stopped_because = "timed out"
					break
				time.sleep(SAMPLE_INTERVAL_SECONDS)
		finally:
			self._write_2_bytes(GOAL_SPEED, 0, "stop")
		time.sleep(COAST_SETTLE_SECONDS)
		servo_steps += steps_moved(previous_position,
		                           self._read_2_bytes(PRESENT_POSITION, "final position"))
		degrees_turned = servo_steps * servo_direction * DEGREES_PER_STEP
		return {
			"direction": "clockwise" if clockwise else "anticlockwise",
			"requested_degrees": target_degrees,
			"degrees_turned": round(degrees_turned, 1),   # in the requested direction
			"stopped_because": stopped_because,
			"peak_load_percent": peak_load_percent,
			"seconds": round(time.monotonic() - started, 2),
			"sample_columns": ["seconds", "degrees_turned", "load_percent"],
			"samples": samples,
		}

	def _stop_running_rotation(self):
		"""Ask any running rotation to stop and wait until it has. The stop
		flag is only cleared once it has finished, so it cannot be missed."""
		self._stop_requested.set()
		self._rotation_finished.wait(timeout=self.config.max_rotation_seconds + 2)
		self._stop_requested.clear()

	def _begin_rotation(self):
		self._stop_running_rotation()
		self._rotation_finished.clear()

	def rotate_by_degrees(self, degrees, speed_rpm=None, stop_above_load_percent=None):
		"""Turn the bit by `degrees`: positive clockwise (tightens), negative
		anticlockwise (loosens). Waits until it has finished, then returns what
		happened, with the load trace (see the module docstring).

		stop_above_load_percent stops it early once the load passes that, e.g.
		when the bit drops into the screw head and starts to meet resistance."""
		speed_rpm = self._speed_rpm_or_default(speed_rpm)
		stop_above_load_percent = self._stop_above_load_or_none(stop_above_load_percent)
		self._begin_rotation()
		try:
			result = self._run_rotation(clockwise=degrees > 0, speed_rpm=speed_rpm,
			                            target_degrees=abs(degrees),
			                            max_seconds=self.config.max_rotation_seconds,
			                            stop_above_load_percent=stop_above_load_percent)
		finally:
			self._rotation_finished.set()
		result["requested_degrees"] = degrees
		self.last_rotation = result
		return result

	def rotate(self, direction, speed_rpm=None, max_seconds=None):
		"""Start turning "clockwise" or "anticlockwise" and return straight
		away. It keeps turning until stop(), the load limit, or max_seconds
		(at most the config's max_rotation_seconds)."""
		if direction not in ("clockwise", "anticlockwise"):
			raise ValueError('direction must be "clockwise" or "anticlockwise"')
		speed_rpm = self._speed_rpm_or_default(speed_rpm)
		max_seconds = self.config.max_rotation_seconds if max_seconds is None \
			else min(float(max_seconds), self.config.max_rotation_seconds)
		self._begin_rotation()

		def run():
			try:
				self.last_rotation = self._run_rotation(
					clockwise=direction == "clockwise", speed_rpm=speed_rpm,
					target_degrees=None, max_seconds=max_seconds)
			except Exception as exc:
				self.last_rotation = {"direction": direction, "stopped_because": "error: %s" % exc}
			finally:
				self._rotation_finished.set()

		threading.Thread(target=run, daemon=True).start()
		return {"direction": direction, "speed_rpm": speed_rpm, "stops_after_seconds": max_seconds}

	def stop(self):
		"""Stop any rotation. Returns the result of the one that was running."""
		self._stop_running_rotation()
		self._write_2_bytes(GOAL_SPEED, 0, "stop")
		return self.last_rotation

	def is_rotating(self):
		return not self._rotation_finished.is_set()

	def status(self):
		"""What the servo is doing now, plus a summary of the last rotation."""
		last_rotation_summary = None
		if self.last_rotation is not None:
			last_rotation_summary = {key: value for key, value in self.last_rotation.items()
			                         if key not in ("samples", "sample_columns")}
		return {
			"rotating": self.is_rotating(),
			"position": self._read_2_bytes(PRESENT_POSITION, "position"),
			"speed_steps_per_second": decode_sign(
				self._read_2_bytes(PRESENT_SPEED, "speed"), SPEED_SIGN_BIT),
			"load_percent": self._read_load_percent(),
			"current_raw": self._read_2_bytes(PRESENT_CURRENT, "current"),
			"voltage": self._read_1_byte(PRESENT_VOLTAGE, "voltage") / 10,
			"temperature_c": self._read_1_byte(PRESENT_TEMPERATURE, "temperature"),
			"last_servo_error": self.last_servo_error,
			"last_rotation": last_rotation_summary,
		}

	## Stop the motor and release the port. Safe to call more than once,
	## and on a controller whose connection failed part-way.
	def close(self):
		port = getattr(self, "_port", None)
		if port is None:
			return
		try:
			if getattr(self, "_connected", False):
				self._stop_running_rotation()
				self._write_2_bytes(GOAL_SPEED, 0, "stop")
		except Exception:
			pass
		finally:
			self._connected = False
			self._port = None
			try:
				port.closePort()
			except Exception:
				pass                        # it never opened

	def __del__(self):
		self.close()
