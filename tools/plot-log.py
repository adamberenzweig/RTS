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

def ReadLog(filename):
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
      if len(fields) == 5:
        data.append(tuple([int(f) for f in fields]))
  fp.close()
  D = np.array(data)
  return D

infile = sys.argv[1]
D = ReadLog(infile)
print D.shape

ax1 = plt.subplot(111)
p1, = plt.plot(D[:,0]/float(1000*60*60), D[:,2], label='volt')
# twinx shares the x but allows independent y.
ax2 = ax1.twinx()
p2, = ax2.plot(D[:,0]/float(1000*60*60), D[:,3], 'r', label='solar')
plots = [p1, p2]
plt.legend(plots, [p.get_label() for p in plots])
# FIXME: This disappears:
plt.xlabel('hours')
plt.show()
