#!/usr/bin/python
# Author: Adam Berenzweig (adam.b@gmail.com)
#
# Analyze the Mega log data.

import collections
import sys

MIN_ID = 2
MAX_ID = 201

def ReadLog(infile):
  fp = open(infile, 'r')
  loglines = collections.defaultdict(list)
  maybe_status = False
  for line in fp:
    if maybe_status:
      pieces = line.split()
      if pieces[0] == 'M' and int(pieces[1]) < MAX_ID:
        loglines[int(pieces[1])].append(line)
    maybe_status = line.startswith('M Status')
  fp.close()
  return loglines


def mean(x):
  if not x:
    return 0.0
  return sum(x) / float(len(x))


def SummarizeOne(logs):
  bad = []
  low = []
  lonely = []
  volts = []
  for log in logs:
    pieces = log.split()
    if len(pieces) != 10:
      continue
    (M, id, total_ms, sleep_ms, nbad, nlonely, nlow, solar, state, voltage) = (
      pieces)
    bad.append(int(nbad))
    low.append(int(nlow))
    lonely.append(int(nlonely))
    volts.append(float(voltage))
  summary = (max(bad), max(low), max(lonely),
             min(volts), mean(volts), len(logs))
  return summary


def SummarizeAll(loglines):
  summaries = {}
  for id, logs in loglines.items():
    summaries[id] = SummarizeOne(logs)
  return summaries


def GetExtremes(summaries):
  bad = []
  lonely = []
  low = []
  num_statuses = []
  for id, summary in summaries.items():
    bad.append(summary[0])
    lonely.append(summary[1])
    low.append(summary[2])
    num_statuses.append(summary[5])
  return (float(max(bad)), float(max(lonely)), float(max(low)),
          float(max(num_statuses)))


def RelativePenalties(summary, extremes):
  penalty = 0.0
  # TODO Or should we have a fixed penalty for any low_v above a threshold?
  # bad, lonely, low: smaller is better.
  for i in (0, 1, 2):
    penalty += summary[i] / extremes[i]
  # num_status, high is better.
  penalty += 1.0 - summary[5] / extremes[3]
  return penalty


def Score(summaries, extremes):
  results = []
  for id, summary in summaries.items():
    # min volts is base.
    score = summary[3]
    score -= RelativePenalties(summary, extremes)
    results.append((score, id, summary))
  return results
    

if len(sys.argv) > 1:
  infile = sys.argv[1]
else:
  print 'Reading from stdin...'
  infile = sys.stdin

loglines = ReadLog(infile)
summaries = SummarizeAll(loglines)
#print summaries
extremes = GetExtremes(summaries)
scored_results = Score(summaries, extremes)
for (score, id, summary) in sorted(scored_results, reverse=True):
  print id, '%.2f' % score, summary
