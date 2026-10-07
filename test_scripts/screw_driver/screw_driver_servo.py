from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS
import time

PORT = "COM4"
BAUD = 1000000
SERVO_ID = 1
PROTOCOL_END = 0
STEPS_PER_TURN = 4096
HALF_TURN = STEPS_PER_TURN // 2

# Request codes
PRESENT_POSITION = 56   # 2 bytes, 0-4095 for one turn
PRESENT_SPEED    = 58   # 2 bytes, steps per second, sign in bit 15
PRESENT_LOAD     = 60   # 2 bytes, tenths of a % of max torque, sign in bit 10
PRESENT_VOLTAGE  = 62   # 1 byte, tenths of a volt
PRESENT_TEMP     = 63   # 1 byte, degrees C

# Settings
MODE           = 33   # 1 byte: 0 = position, 1 = wheel (continuous)
TORQUE_ENABLE  = 40   # 1 byte: 1 = holds and drives, 0 = goes limp
ACCELERATION   = 41   # 1 byte: 0-254, in units of 100 steps/s per second
GOAL_POSITION  = 42   # 2 bytes, 0-4095
GOAL_SPEED     = 46   # 2 bytes, steps per second, sign in bit 15 (wheel mode)
TORQUE_LIMIT   = 48   # 2 bytes, tenths of a % of max torque (1000 = 100%)

def check(comm, err, what):
    if comm != COMM_SUCCESS:
        raise RuntimeError(f"no reply to {what}: {packetHandler.getTxRxResult(comm)}")
    if err:
        print(f"servo flagged an error during {what}:", packetHandler.getRxPacketError(err))

def steps_moved(previous, current):
	# returns the number of steps moved from previous to current, taking into account wraparound
	delta = current - previous
	if delta > HALF_TURN:
		delta -= STEPS_PER_TURN
	elif delta < -HALF_TURN:
		delta += STEPS_PER_TURN
	return delta

def decode_sign(raw, sign_bit):
    # Sign bit for speed is 15, for load is 10
    size = raw & ((1 << sign_bit) - 1)        # keep only the bits below the sign bit
    is_negative = (raw & (1 << sign_bit))  # check if the sign bit is set
    return -size if is_negative else size

def encode_sign(value, sign_bit):
    return (-value) | (1 << sign_bit) if value < 0 else value

def read_1_byte(address, what):
    value, comm, err = packetHandler.read1ByteTxRx(port, SERVO_ID, address)
    check(comm, err, what)
    return value

def read_2_bytes(address, what):
    value, comm, err = packetHandler.read2ByteTxRx(port, SERVO_ID, address)
    check(comm, err, what)
    return value

def write_1_byte(address, value, what):
    comm, err = packetHandler.write1ByteTxRx(port, SERVO_ID, address, value)
    check(comm, err, what)

def write_2_bytes(address, value, what):
    comm, err = packetHandler.write2ByteTxRx(port, SERVO_ID, address, value)
    check(comm, err, what)


### Actions
def move_to(target_position, speed=800):
	write_1_byte(MODE, 0, "mode")
	write_2_bytes(TORQUE_LIMIT, 300, "torque limit")          # 30% while learning
	write_1_byte(ACCELERATION, 50, "acceleration")
	write_2_bytes(GOAL_SPEED, speed, "speed")
	write_2_bytes(GOAL_POSITION, target_position, "goal position")   # this starts the move
	while abs(read_2_bytes(PRESENT_POSITION, "position") - target_position) > 10:
		time.sleep(0.02)

def spin_for(seconds, speed=500):
	write_1_byte(MODE, 1, "mode")
	write_2_bytes(TORQUE_LIMIT, 300, "torque limit")
	try:
		write_2_bytes(GOAL_SPEED, encode_sign(speed, 15), "speed")
		time.sleep(seconds)
	finally:
		write_2_bytes(GOAL_SPEED, 0, "stop")

### Monitoring
def monitor():
	print("monitoring servo, press Ctrl-C to stop")
	print("pos  speed   load  volts  temp")
	print("---- ------ ------ ----- ----")
	while True:
		position = read_2_bytes(PRESENT_POSITION, "position")
		speed = decode_sign(read_2_bytes(PRESENT_SPEED, "speed"), 15)
		load = decode_sign(read_2_bytes(PRESENT_LOAD, "load"), 10) / 10
		volts = read_1_byte(PRESENT_VOLTAGE, "voltage") / 10
		temp = read_1_byte(PRESENT_TEMP, "temperature")
		print(f"pos {position:4d}  speed {speed:5d}  load {load:+6.1f}%  "
				f"{volts:.1f} V  {temp} C")
		time.sleep(0.2)

port = PortHandler(PORT)
packetHandler = PacketHandler(PROTOCOL_END)

if not port.openPort():
    raise SystemExit(f"could not open {PORT}")
if not port.setBaudRate(BAUD):
    raise SystemExit(f"could not set baudrate to {BAUD}")

model, comm, err = packetHandler.ping(port, SERVO_ID)
check(comm, err, "ping")
print(f"servo {SERVO_ID} answered, model number {model}")

try:
	move_to(2048) # half a turn
	move_to(0)
	spin_for(3)         # three seconds forward
	spin_for(3, -500)   # three seconds back
	monitor()             # then show readings until Ctrl+C
except KeyboardInterrupt:
    raise SystemExit("stopped")
finally:
    port.closePort()