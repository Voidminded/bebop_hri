#!/usr/bin/env python

import csv
import sys

fname = sys.argv[1]+"_status.csv"
f = open(fname, 'rb')
reader = csv.reader(f)
startTime = -1.0;
finishTime = -1.0;
for row in reader:
    if ( startTime == -1 and row[1] == "Current State: Approaching The person"):
        startTime = row[0]
f.close()
fname = sys.argv[1]+"_land.csv"
f = open(fname, 'rb')
reader = csv.reader(f)
for row in reader:
    if row[0] > startTime:
        finishTime = row[0]
        break
f.close()
transitions = []
fname = sys.argv[1]+"_bebop.csv"
f = open(fname, 'rb')
reader = csv.reader(f)
for row in reader:
    if startTime <= row[2] <= finishTime:
        transitions.append([row[2],row[5],row[6],row[7]])
fname = sys.argv[1]+"_bebop_trimmed.csv"
f = open(fname, 'w')
writer = csv.writer(f, delimiter=',')
for row in transitions:
    writer.writerow(row)
f.close()
trans = []
fname = sys.argv[1]+"_target.csv"
f = open(fname, 'rb')
reader = csv.reader(f)
for row in reader:
    if startTime <= row[2] <= finishTime:
        trans.append([row[2],row[5],row[6],row[7]])
fname = sys.argv[1]+"_target_trimmed.csv"
f = open(fname, 'w')
writer = csv.writer(f, delimiter=',')
for row in trans:
    writer.writerow(row)
f.close()
