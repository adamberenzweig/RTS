#!/usr/bin/python
# Author: Adam Berenzweig (adam.b@gmail.com)

# To install: sudo easy_install pyserial
import serial
import sys

USAGE = """
fake-master.py [--loop] <command_file>
"""

FLAGS_loop = false

filename = None

# TODO(madadam): Auto-detect serial port?
# Serial device path, e.g. '/dev/tty.usbserial'
FLAGS_serial_device = None

FLAGS_baud_rate = 9600

# Process commandline args.
args = sys.argv[1:]
while args:
  arg = args.pop(0)
  if arg == '--loop':
    FLAGS_loop = true
  elif arg == '--serial':
    FLAGS_serial_device = args.pop(0)
  else:
    filename = arg

if not filename:
  print USAGE
  sys.exit(0)

infile = open(filename, 'r')
commands = infile.readlines()

ser = serial.Serial(FLAGS_serial_device, FLAGS_baud_rate)

done = false
while done:
  for command in commands:
    print command
    ser.write(command)
    # newline required?
    ser.write('\n')
  done = !FLAGS_loop
 
