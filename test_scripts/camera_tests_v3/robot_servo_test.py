import os
import sys
import time
import functools
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../'))
from uarm.wrapper import SwiftAPI

swift = SwiftAPI(filters={'hwid': 'USB VID:PID=2341:0042'})

swift.waiting_ready()

swift.set_wrist(130)
time.sleep(1)
swift.set_wrist(150)
time.sleep(1)
swift.set_wrist(200)
time.sleep(1)
swift.set_wrist(90)

time.sleep(4)
swift.flush_cmd()
swift.disconnect()
