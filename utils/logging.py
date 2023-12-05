from time import time, ctime

log_name = "log_file_"
log_extension = ".log"


def write_log(writer=None, write_msg=None):
	if writer is not None:
		if write_msg is not None:
			f = open(log_name + writer + log_extension, "a")
			f.write("\n" + "{}: {}".format(ctime(time()), write_msg))
			f.close()
		else:
			print("Please state a message to write to the log file")
	else:
		print("Please state: client or server")
