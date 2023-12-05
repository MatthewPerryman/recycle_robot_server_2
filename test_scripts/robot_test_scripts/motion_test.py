from uarm.wrapper import SwiftAPI
import keyboard

swift = SwiftAPI()

connected = swift.connect()
if not connected:
    print("Failed to connect to uArm robotic arm")
    exit(1)

def print_position():
    x, y, z = swift.get_position()
    print(f"Current position: x={x:.2f}, y={y:.2f}, z={z:.2f}")

def move_up():
    x, y, z = swift.get_position()
    swift.set_position(x, y, z + 1)
    print_position()

def move_down():
    x, y, z = swift.get_position()
    swift.set_position(x, y, z - 1)
    print_position()

def move_left():
    x, y, z = swift.get_position()
    swift.set_position(x - 1, y, z)
    print_position()

def move_right():
    x, y, z = swift.get_position()
    swift.set_position(x + 1, y, z)
    print_position()

def move_forwards():
    x, y, z = swift.get_position()
    swift.set_position(x, y + 1, z)
    print_position()

def move_backwards():
    x, y, z = swift.get_position()
    swift.set_position(x, y - 1, z)
    print_position()

keyboard.add_hotkey('up', move_up)
keyboard.add_hotkey('down', move_down)
keyboard.add_hotkey('left', move_left)
keyboard.add_hotkey('right', move_right)
keyboard.add_hotkey('w', move_forwards)
keyboard.add_hotkey('s', move_backwards)

keyboard.wait('esc')
