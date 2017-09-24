# Converts an RTKLIB .pos file into .csv

import matplotlib.dates as mdates
import sys
import os
import string
import csv

print 'POS to CSV'

filename = ''

if filename == '':
    # Check if the pos filename was passed in argv
    if len(sys.argv) > 1: filename = sys.argv[1]

firstfile = ''
for root, dirs, files in os.walk("."):
    if len(files) > 0:
        if root == ".":
            for afile in files:
                if afile[-4:] == '.pos':
                    if firstfile == '': firstfile = afile

if filename == '': filename = raw_input('Enter the .pos filename (default: ' + firstfile + '): ') # Get the filename
if filename == '': filename = firstfile

print 'Processing',filename
filenamestem = filename[:-4]
outfile = filenamestem + '.csv'
print 'Writing to',outfile

try:
    fi = open(filename,"r")
except:
    raise Exception('Invalid input file!')

try:
    fo = open(outfile,"wb")
except:
    raise Exception('Invalid output file!')
output=csv.writer(fo,delimiter=',')

lines = 0
ignored = 0

try:
    for line in fi:
        if line[0] != '%': # ignore header lines which all start '% '
            fields = string.split(line)
            if fields[5] == '1': # Check the Q value
                # Only convert data with a Q of 1
                time_str = fields[0] + ' ' + fields[1]
                #output.writerow([time_str, fields[2], fields[3], fields[4]])
                output.writerow([fields[2], fields[3], fields[4]])
                lines += 1
            else:
                ignored += 1

    print 'Processed',lines,'data points'
    print 'Ignored',ignored,'data points'

finally: # Close the files
    try:
        fi.close()
    except:
        pass
    try:
        fo.close()
    except:
        pass
    print 'Bye!'




