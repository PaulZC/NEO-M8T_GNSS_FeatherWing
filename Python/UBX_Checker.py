# Checks the format of u-blox binary files

import sys
import os

def csum(byte, sum1, sum2):
    sum1 = sum1 + byte
    sum2 = sum2 + sum1
    sum1 = sum1 & 0xFF
    sum2 = sum2 & 0xFF
    return sum1,sum2

print 'UBX Binary File Checker'

filename = ''

if filename == '':
    # Check if the bin filename was passed in argv
    if len(sys.argv) > 1: filename = sys.argv[1]

firstfile = ''
for root, dirs, files in os.walk("."):
    if len(files) > 0:
        if root == ".":
            for afile in files:
                if afile[-4:] == '.BIN':
                    if firstfile == '': firstfile = afile

if filename == '': filename = raw_input('Enter the bin filename (default: ' + firstfile + '): ') # Get the filename
if filename == '': filename = firstfile

print 'Processing',filename
filesize = os.path.getsize(filename)

try:
    fi = open(filename,"rb")
except:
    raise Exception('Invalid file!')

processed = 0
messages = 0

try:
    while True:
        sum1 = 0
        sum2 = 0
        print
        result = fi.read(2)
        if len(result) == 0:
            raise Exception('End of file?!')
        if len(result) != 2:
            raise Exception('Failed to read header bytes!')
        if (ord(result[0]) != 0xb5) or (ord(result[1]) != 0x62):
            raise Exception('Failed to read valid header bytes!')
        processed += 2
        result = fi.read(2)
        if len(result) != 2:
            raise Exception('Failed to read type bytes!')
        print 'Processing message type 0x%02X 0x%02X'%(ord(result[0]),ord(result[1]))
        sum1,sum2 = csum(ord(result[0]),sum1,sum2)
        sum1,sum2 = csum(ord(result[1]),sum1,sum2)
        processed += 2
        result = fi.read(2)
        if len(result) != 2:
            raise Exception('Failed to read length bytes!')
        length = (ord(result[1]) * 256) + ord(result[0])
        print 'Message contains %i (0x%04X) data bytes'%(length,length)
        sum1,sum2 = csum(ord(result[0]),sum1,sum2)
        sum1,sum2 = csum(ord(result[1]),sum1,sum2)
        processed += 2
        for l in range(length):
            byte = fi.read(1)
            if byte == '':
                raise Exception('Failed to read data byte!')
            sum1,sum2 = csum(ord(byte),sum1,sum2)
            processed += 1
        result = fi.read(2)
        if len(result) != 2:
            raise Exception('Failed to read checksum bytes!')
        if (ord(result[0]) != sum1) or (ord(result[1]) != sum2):
            print 'Checksum failure! Expected 0x%02X 0x%02X : Got 0x%02X 0x%02X'%(sum1,sum2,ord(result[0]),ord(result[1]))
            #raise Exception('Checksum failure!')
        processed += 2
        messages += 1

finally:
    fi.close()
    print
    print 'Processed',processed,'bytes in',messages,'messages'
    print 'File size was',filesize
    if (processed != filesize):
        print 'FILE SIZE MISMATCH!!'
    print 'Bye!'
