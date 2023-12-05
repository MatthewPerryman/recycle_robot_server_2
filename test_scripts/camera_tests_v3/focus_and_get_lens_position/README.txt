This code was to get the lens position of the camera from picamera2 libcamera when the cameras autofocus has finished.
This gives me the settings for focussing the camera manually for a given height of the robot.
However, it was created with assumption that I could in any way obtain the an accurate focal length of the camera for objects closer than infite.
This is not possible using lib camera library and with this camera/pi config.
