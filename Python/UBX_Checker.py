# Checks the format of u-blox binary files

import sys
import os

# Add byte to checksums sum1 and sum2
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

# Find first .bin file in the current directory
firstfile = ''
for root, dirs, files in os.walk("."):
    if len(files) > 0:
        if root == ".": # Comment this line to check sub-directories too
            for afile in files:
                if afile[-4:] == '.bin':
                    if firstfile == '': firstfile = afile

# Ask user for .bin filename offering firstfile as the default
if filename == '': filename = raw_input('Enter the bin filename (default: ' + firstfile + '): ') # Get the filename
if filename == '': filename = firstfile

print 'Processing',filename
filesize = os.path.getsize(filename)

# Try to open file for reading
try:
    fi = open(filename,"rb")
except:
    raise Exception('Invalid file!')

processed = 0
messages = {}
longest = 0

try:
    while True:
        sum1 = 0 # Checksum bytes
        sum2 = 0

        # Try to read two bytes, check they are the two sync chars
        print
        result = fi.read(2)
        if len(result) == 0:
            raise Exception('End of file?!')
        if len(result) != 2:
            raise Exception('Failed to read header bytes!')
        if (ord(result[0]) != 0xb5) or (ord(result[1]) != 0x62):
            raise Exception('Failed to read valid header bytes!')
        processed += 2

        # Try to read two more bytes, these will be the type bytes
        result = fi.read(2)
        if len(result) != 2:
            raise Exception('Failed to read type bytes!')
        message_type = '0x%02X 0x%02X'%(ord(result[0]),ord(result[1]))
        print 'Processing message type',message_type
        sum1,sum2 = csum(ord(result[0]),sum1,sum2)
        sum1,sum2 = csum(ord(result[1]),sum1,sum2)
        processed += 2

        # Try to read two more bytes, these will be the message length
        result = fi.read(2)
        if len(result) != 2:
            raise Exception('Failed to read length bytes!')
        length = (ord(result[1]) * 256) + ord(result[0])
        print 'Message contains %i (0x%04X) data bytes'%(length,length)
        sum1,sum2 = csum(ord(result[0]),sum1,sum2)
        sum1,sum2 = csum(ord(result[1]),sum1,sum2)
        processed += 2

        # Try to read length bytes
        for l in range(length):
            byte = fi.read(1)
            if byte == '':
                raise Exception('Failed to read data byte!')
            sum1,sum2 = csum(ord(byte),sum1,sum2)
            processed += 1

        # Try to read two checksum bytes
        result = fi.read(2)
        if len(result) != 2:
            raise Exception('Failed to read checksum bytes!')
        if (ord(result[0]) != sum1) or (ord(result[1]) != sum2):
            print 'Checksum failure! Expected 0x%02X 0x%02X : Got 0x%02X 0x%02X'%(sum1,sum2,ord(result[0]),ord(result[1]))
            #raise Exception('Checksum failure!')
        processed += 2

        # Check if we have seen this message type before
        if messages.has_key(message_type):
            messages[message_type] += 1 # if we have, increment its count
        else:
            messages[message_type] = 1 # if we have not, set its count to 1

        # Update the longest message length
        if (length > longest): longest = length

finally:
    fi.close() # Close the file

    # Print the file statistics
    print
    print 'Processed',processed,'bytes'
    print 'File size was',filesize
    if (processed != filesize):
        print 'FILE SIZE MISMATCH!!'
    print 'Longest message was %i data bytes'%longest
    if len(messages) > 0:
        print 'Message types and totals were:'
        for key in messages.keys():
            print 'Message type:',key,'  Total:',messages[key]

    print 'Bye!'
