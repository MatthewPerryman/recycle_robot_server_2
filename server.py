from server_package.api.api import app, controller, image_stream


if __name__ == "__main__":
	try:
		app.run(port=1024, host="0.0.0.0")
	finally:
		# runs on Ctrl+C and on any unhandled error, not just KeyboardInterrupt
		image_stream.close()
		controller.close()
