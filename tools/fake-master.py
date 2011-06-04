#!/usr/bin/python
# Author: Adam Berenzweig (adam.b@gmail.com)
#
# A tool to drive the master by sending it commands over the USB port.
# With the master hooked up to a laptop via the USB programmer, run this
# to send commands from a text file.  Useful for testing different patterns
# without having to reprogram the master every time.

# To install serial: sudo easy_install pyserial
import serial
import sys
import time

USAGE = """
fake-master.py [options] <command_file>

Options:
--loop: If true, loop through the file forever.
--serial_device: Serial device to communicate to Arduino,
                 e.g. /dev/tty.usbserial-<id>
"""

FLAGS_loop = False

# TODO(madadam): Auto-detect serial port?
# Serial device path, e.g. '/dev/tty.usbserial'
FLAGS_serial_device = None

FLAGS_baud_rate = 9600

filename = None

# Process commandline args.
args = sys.argv[1:]
while args:
  arg = args.pop(0)
  if '=' in arg:
    arg, arg_value = arg.split('=')
  if arg == '--loop':
    FLAGS_loop = True
  elif arg == '--serial_device':
    FLAGS_serial_device = arg_value
  else:
    filename = arg

if not filename:
  print USAGE
  sys.exit(0)

infile = open(filename, 'r')
command_lines = infile.readlines();
infile.close()

ser = serial.Serial(FLAGS_serial_device, FLAGS_baud_rate)

done = False
while not done:
  for line in command_lines:
    duration_ms, command = line.strip().split(None, 1)
    duration_sec = int(duration_ms) / 1000.0
    print command
    ser.write(command)
    # newline required?
    ser.write('\n')
    time.sleep(duration_sec)
  done = not FLAGS_loop
 
