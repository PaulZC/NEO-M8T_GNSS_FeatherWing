## U-Blox NEO-M8T GNSS RAWX Logger

## Hardware:
## Paul's NEO-M8T GNSS FeatherWing:
## https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing
## Mounted on the Adafruit Feather M0 Adalogger:
## https://www.adafruit.com/product/2796
## https://learn.adafruit.com/adafruit-feather-m0-adalogger

## Assumes that the Adalogger is running UBX_Echo
## which sets the NEO-M8T GNSS Baud rate to 115200
## https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/tree/master/Arduino/UBX_Echo

import serial
import time
import sys

class UBXport(object):

    def __init__(self):
        ''' Init UBXport - open the serial port '''
        com_port = raw_input('Which serial port do you want to use (default COM1)? ')
        if com_port == '': com_port = 'COM1'

        # Open port
        try:
            self.ser1 = serial.Serial(com_port, 115200, timeout=0.1)
        except:
            raise NameError('COULD NOT OPEN SERIAL PORT!')

        self.ser1.flushInput()

    def sendNMEA(self,msg,wait=10,biglen=200):
        ''' Send a message in NMEA format (adding $ and *, the checksum, and \r\n) and wait for a reply '''
        a = []
        a += "$" + msg + "*"
        csum = 0 
        for c in a[1:-1]: csum ^= ord(c)        
        a += "%02X"%csum + '\r\n'
        print 'Sending:'
        print ''.join(a[:-2])
        for c in a: self.ser1.write(c)
        print 'Received:'
        for x in range(wait):
            rx = up.ser1.read(biglen)
            if rx != '': sys.stdout.write(rx)

    def sendUBX(self,msg,wait=10,biglen=200):
        ''' Send a message in UBX format (adding the checksum) and wait for a reply '''
        a = []
        a += msg
        sum1 = 0
        sum2 = 0
        for c in a[2:]:
            sum1 = sum1 + ord(c)
            sum2 = sum2 + sum1
        a += chr(sum1&0xFF)
        a += chr(sum2&0xFF)
        print 'Sending:'
        print "\\x" + "\\x".join("{:02x}".format(ord(c)) for c in a)
        for c in a: self.ser1.write(c)
        print 'Received:'
        for x in range(wait):
            rx = self.ser1.read(biglen)
            if rx != '':
                print "\\x" + "\\x".join("{:02x}".format(ord(c)) for c in rx)
    

try:
    print 'NEO-M8T GNSS RAWX Logger'
    print
    
    up = UBXport() # Open port

    # Disable all default NMEA messages
    print 'Disabling GLL'
    up.sendNMEA("PUBX,40,GLL,0,0,0,0")
    print 'Disabling ZDA'
    up.sendNMEA("PUBX,40,ZDA,0,0,0,0")
    print 'Disabling VTG'
    up.sendNMEA("PUBX,40,VTG,0,0,0,0")
    print 'Disabling GSV'
    up.sendNMEA("PUBX,40,GSV,0,0,0,0")
    print 'Disabling GSA'
    up.sendNMEA("PUBX,40,GSA,0,0,0,0")
    print 'Disabling RMC'
    up.sendNMEA("PUBX,40,RMC,0,0,0,0")       
    print 'Disabling GGA'
    up.sendNMEA("PUBX,40,GGA,0,0,0,0")       

    # Set Navigation Mode
    print
    print 'Setting Nav Mode'
    print '\\xb5\\x62\\x05\\x01... indicates ACK'
    print '\\xb5\\x62\\x05\\x00... indicates NACK'
    # Get Navigation Engine
    #up.sendUBX("\xB5\x62\x06\x24\x00\x00")
    # Set Navigation Engine dynModel to "Portable"
    # Set tAcc to 300
    # Set dgnssTimeout to zero
    up.sendUBX("\xB5\x62\x06\x24\x24\x00\xFF\xFF\x00\x03\x00\x00\x00\x00\x10\x27\x00\x00\x05\x00\xFA\x00\xFA\x00\x64\x00\x2C\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00")
    # Set Navigation Engine dynModel to "Pedestrian"
    #up.sendUBX("\xB5\x62\x06\x24\x24\x00\xFF\xFF\x03\x03\x00\x00\x00\x00\x10\x27\x00\x00\x05\x00\xFA\x00\xFA\x00\x64\x00\x2C\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00")
    # Set Navigation Engine dynModel to "Automotive"
    #up.sendUBX("\xB5\x62\x06\x24\x24\x00\xFF\xFF\x04\x03\x00\x00\x00\x00\x10\x27\x00\x00\x05\x00\xFA\x00\xFA\x00\x64\x00\x2C\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00")
    # Set Navigation Engine dynModel to "Sea"
    #up.sendUBX("\xB5\x62\x06\x24\x24\x00\xFF\xFF\x05\x03\x00\x00\x00\x00\x10\x27\x00\x00\x05\x00\xFA\x00\xFA\x00\x64\x00\x2C\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00")
    # Set Navigation Engine dynModel to "Airborne <1G"
    #up.sendUBX("\xB5\x62\x06\x24\x24\x00\xFF\xFF\x06\x03\x00\x00\x00\x00\x10\x27\x00\x00\x05\x00\xFA\x00\xFA\x00\x64\x00\x2C\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00")

    # Config NMEA Talker ID
    print
    print 'Setting NMEA Config'
    # Get NMEA
    #up.sendUBX("\xb5\x62\x06\x17\x00\x00")
    # Set NMEA Config:
    # Set trackFilt to 1 to ensure course (COG) is always output
    # Set Main Talker ID to 'GP' to avoid having to modify TinyGPS
    up.sendUBX("\xb5\x62\x06\x17\x14\x00\x20\x40\x00\x02\x00\x00\x00\x00\x00\x01\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00")
    # Set Main Talker ID to 'GN'
    #up.sendUBX("\xb5\x62\x06\x17\x14\x00\x20\x40\x00\x02\x00\x00\x00\x00\x00\x03\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00")

    # Change Navigation/Measurement Rate
    print
    print 'Setting Measurement Rate'
    # Get CFG-RATE
    #up.sendUBX("\xb5\x62\x06\x08\x00\x00")
    # Set CFG-RATE: set measRate to 200msec; align to UTC time
    up.sendUBX("\xb5\x62\x06\x08\x06\x00\xc8\x00\x01\x00\x00\x00")
    # Set CFG-RATE: set measRate to 200msec; align to GPS time
    #up.sendUBX("\xb5\x62\x06\x08\x06\x00\xc8\x00\x01\x00\x01\x00")

    # Set RAWX message rate (once per measRate)
    print
    print 'Setting RXM-RAWX Message Rate'
    # Poll RXM-RAWX send rate
    #up.sendUBX("\xb5\x62\x06\x01\x02\x00\x02\x15")
    # Set RXM-RAWX send rate
    up.sendUBX("\xb5\x62\x06\x01\x03\x00\x02\x15\x01",1,10) # current port
    #up.sendUBX("\xb5\x62\x06\x01\x08\x00\x02\x15\x00\x01\x00\x00\x00\x00",1,10) # six ports

##    # Enble the NAV-PVT message - useful for testing on SAM-M8Q or MAX-M8Q which don't support RAWX
##    print
##    print 'Setting NAV-PVT Message Rate'
##    # Poll NAV-PVT send rate
##    #up.sendUBX("\xb5\x62\x06\x01\x02\x00\x01\x07")
##    # Set NAV-PVT send rate
##    up.sendUBX("\xb5\x62\x06\x01\x03\x00\x01\x07\x01",1,10) # current port
##    #up.sendUBX("\xb5\x62\x06\x01\x08\x00\x01\x07\x00\x01\x00\x00\x00\x00",1,10) # six ports

    start_time = time.time() # Get the time
    tn = time.localtime(start_time) # Extract the time and date as strings
    date_str = str(tn[0])+str(tn[1]).zfill(2)+str(tn[2]).zfill(2)
    time_str = str(tn[3]).zfill(2)+str(tn[4]).zfill(2)+str(tn[5]).zfill(2)
    # Assemble the file name using the date, time and port name
    filename = 'GNSS_RAWX_Log_' + date_str + '_' + time_str + '.bin'
    print
    print 'Logging data to', filename
    print
    print 'Press CTRL+C to stop logging'
    print

    fp = open(filename, 'wb') # Create / clear the file

    while True:
        rx = up.ser1.read(200) # Read serial data forcing a timeout
        if rx != '': # if we got some data:
            # print it in Python hex syntax (useful for cutting and pasting)
            rx_str = "\\x" + "\\x".join("{:02x}".format(ord(c)) for c in rx)
            sys.stdout.write(rx_str)
            # and write it to the file
            fp.write(rx)

except KeyboardInterrupt:
    print
    print 'CTRL+C received...'

finally:
    up.ser1.close() # Close the serial port
    fp.close() # Close the file


