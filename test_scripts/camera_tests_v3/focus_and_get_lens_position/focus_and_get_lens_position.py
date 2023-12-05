# This code was to get the lens position of the camera from picamera2 libcamera
# when the cameras autofocus has finished.
# This gives me the settings for focussing the camera manually for a given height
# of the robot. However, it was created with assumption that I could in any way
# obtain the an accurate focal length of the camera for objects closer than infite.
# This is not possible using lib camera library and with this camera/pi config.

from picamera2 import Picamera2, Preview
from libcamera import controls
from uarm.wrapper import SwiftAPI
import keyboard

import time

def print_position(swift, location_caption=""):
    x, y, z = swift.get_position()
    arm_location = f"x={x:.2f}, y={y:.2f}, z={z:.2f}"
    if len(location_caption) == 0:
        print("Arm Location: " + arm_location)
    else:
        print(location_caption + arm_location)

## Clear cmd buffer and close connection
def end_transmission(swift):
    swift.flush_cmd(wait_stop=True)
    swift.waiting_ready(timeout=3)

# TODO: Configure the robot to go to a standard location
def start_robot():
    swift = SwiftAPI()

    end_transmission(swift)

    swift.set_position(200, 0, 150)
    end_transmission(swift)
    print_position(swift, "Initial Position: ")

    return swift

swift = start_robot()

# Set the camera to autofocus
picam2 = Picamera2()
preview_config = picam2.create_still_configuration(main={"size": (640, 480)})
picam2.configure(preview_config)
picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})

# Take a photo and get the lens position
picam2.start()
time.sleep(2)
# Get focal position
lens_position = picam2.capture_metadata()['LensPosition']

print(lens_position)
picam2.capture_file(f"image_at_{lens_position}.jpg")

# Ask user to check the 


