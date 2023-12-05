from uarm.wrapper import SwiftAPI
import time

swift = SwiftAPI()

swift.waiting_ready()
swift.flush_cmd()

swift.connect()

swift.reset(wait=True, speed=100000)
location = swift.get_position()

print("Arm Position: ", location)

swift.flush_cmd(wait_stop=True)

swift.disconnect()
