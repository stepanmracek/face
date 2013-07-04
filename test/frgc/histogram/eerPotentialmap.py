import sys

bins = 20

f = open(sys.argv[1])

counter = 0
for line in f:
	if (line.strip() == ''): continue
	val = float(line)

	if counter%bins == 0: print
	
	print counter/bins, counter%bins, val
	counter += 1