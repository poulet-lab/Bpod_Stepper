Import("env")

from datetime import datetime
from struct import *
from array import array

idx = int(env.GetProjectOption("custom_version_index"))
arr = array('B', [idx, datetime.now().month])
arr.frombytes(array('H', [datetime.now().year]).tobytes())
ver = int.from_bytes(arr.tobytes(), 'little')
board = env.GetProjectOption("board").upper()

env.Append(CPPDEFINES=[("FIRMWARE_VERSION", ver)])
env.Replace(PROGNAME="Bpod_Stepper-%s.%d.%s" % (datetime.now().strftime("%Y.%m"), idx, board))
