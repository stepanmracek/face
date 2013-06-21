#! /usr/bin/env python2
import numpy

eerForStripes = {}
eerForBins = {}
minEER = 1.0

f = open('histogram-stripes-bins-map')
for line in f:
	items = line.split()
	if len(items) != 3: continue

	stripes = int(items[0])
	bins = int(items[1])
	eer = float(items[2])

	if eer < minEER:
		minEER = eer
		minEERStripes = stripes
		minEERBins = bins

	if not stripes in eerForStripes:
		eerForStripes[stripes] = []
	eerForStripes[stripes].append(eer)

	if not bins in eerForBins:
		eerForBins[bins] = []
	eerForBins[bins].append(eer)

for bins in eerForBins:
	print "Bins:", bins, "Mean EER", numpy.mean(eerForBins[bins]), numpy.std(eerForBins[bins])

print

for stripes in eerForStripes:
	print "Stripes:", stripes, "Mean EER", numpy.mean(eerForStripes[stripes]), numpy.std(eerForStripes[stripes])

print

print minEERStripes, minEERBins, minEER