#!/usr/bin/python
# Author: Adam Berenzweig (adam.b@gmail.com)
#
# Plot the RTS log data.
# Expected format is:
#
# timestamp_ms total_sleep_ms raw_voltage raw_solar last_state

import sys
import numpy as np
import re
from matplotlib import pyplot as plt

def ReadLog(filename, num_fields_expected):
  fp = open(filename, 'r')
  data = []
  # Some munging to get rid of noise in the logs caused by (I think)
  # the chip sleeping before the USB buffer gets flushed.
  control_chars = ''.join(map(unichr, range(0,32) + range(127,255)))
  control_char_re = re.compile('[%s]+' % re.escape(control_chars))
  for line in fp:
    line = control_char_re.sub('\n', line)
    for clean_line in line.split('\n'):
      fields = clean_line.split()
      if len(fields) == num_fields_expected:
        data.append(tuple([float(f) for f in fields]))
  fp.close()
  D = np.array(data)
  return D

class SmoothedThreshold(object):
  def __init__(self, window_length):
    self.window_length = window_length
    self.value = None
    self.last_update = None
    # Amount of time in the window
    self.size = 0.0

  def Update(self, value, now):
    if not self.last_update:
      self.value = value
    else:
      elapsed_time = min(now - self.last_update, self.window_length)

      if self.size < self.window_length:
        self.size += elapsed_time
      self.value = (
          (value * elapsed_time + self.value * (self.size - elapsed_time)) /
          self.size)

    if self.value > 1023:
      print 'ERROR', elapsed_time, self.size, value, now, self.value

    self.last_update = now
    return self.value


if len(sys.argv) > 1:
  infile = sys.argv[1]
else:
  print 'Reading from stdin...'
  infile = sys.stdin
D = ReadLog(infile, 5)
print D.shape

r1 = 320
r2 = 800
fudge_factor = 1.215  # Why isn't voltage divider math working?
voltage_factor = fudge_factor * 1.05 / (1023.0 * (r1/float(r1+r2)))

scaled_time = D[:,0]/float(1000*60*60)

ax1 = plt.subplot(111)
p1, = plt.plot(scaled_time, D[:,2] * voltage_factor, label='volt')

# twinx shares the x but allows independent y.
ax2 = ax1.twinx()
p2, = ax2.plot(scaled_time, D[:,3], 'r', label='solar')

smoother = SmoothedThreshold(1000 * 60 * 30)  # 30 minutes
smoothed_raw_voltage = np.array([smoother.Update(d[2], d[0]) for d in D])
print smoothed_raw_voltage

p1b, = ax1.plot(scaled_time, smoothed_raw_voltage * voltage_factor,
                'g.', label='smoothed')

if len(sys.argv) > 2:
  vcc_data = ReadLog(sys.argv[2], 2)
  p3, = ax1.plot(vcc_data[:,0]/float(1000*60*60), vcc_data[:,1],
                 'g', label='vcc')

plots = [p1, p1b, p2]
plt.legend(plots, [p.get_label() for p in plots])
# FIXME: This disappears:
plt.xlabel('hours')
plt.show()
