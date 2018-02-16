# NEO-M8T GNSS FeatherWing

A clone of the [Adafruit Ultimate GPS FeatherWing](https://www.adafruit.com/product/3133) but with the u-blox NEO-M8T replacing the GlobalTop FGPMMOPA6H. The NEO-M8T can receive signals from GPS, Galileo and either GLONASS or BeiDou concurrently and supports both SBAS and QZSS.
It also provides Multi-GNSS Raw Measurement (RAWX) Data which can be used for Post-Processed Kinematic (PPK) positioning.

![NEO-M8T_FeatherWing](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/NEO-M8T_FeatherWing.JPG)

![NEO-M8T_FeatherWing_Adalogger_1](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/NEO-M8T_FeatherWing_Adalogger_1.JPG)

![NEO-M8T_FeatherWing_Adalogger_2](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/NEO-M8T_FeatherWing_Adalogger_2.JPG)

See [NEO-M8T_GNSS_FeatherWing.pdf](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/NEO-M8T_GNSS_FeatherWing.pdf) for the schematic, layout and Bill Of Materials.

The [Eagle](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/tree/master/Eagle) directory contains the schematic and pcb design files.

The [Arduino](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/tree/master/Arduino) directory contains code for the [Adafruit Feather M0 Adalogger](https://www.adafruit.com/products/2796) which will log RAWX data to SD card.
- https://www.adafruit.com/products/2796
- https://learn.adafruit.com/adafruit-feather-m0-adalogger

[POST_PROCESS.md](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/POST_PROCESS.md) contains results of a first time successful attempt at post-processed kinematic positioning using [rtkexplorer's demo5 version of RTKLIB](http://rtkexplorer.com/downloads/rtklib-code/).
Please refer to the [Precise Positioning Resources](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/LEARN.md#precise-positioning-resources) for more information.

This guide concentrates on the differences between the NEO-M8T FeatherWing and the Adafruit Ultimate GPS FeatherWing. Please refer to Lady Ada's excellent documentation to get started with the Feather family and GPS:
- https://www.adafruit.com/product/3133
- https://learn.adafruit.com/adafruit-ultimate-gps-featherwing

## Power Pins

Like the Ultimate GPS FeatherWing, the NEO-M8T FeatherWing runs from +3.3V power and uses 3V logic. The NEO-M8T is powered directly from the 3V and GND pins on the bottom left of the Feather. Each Adafruit Feather has a regulator to provide clean 3V power from USB or battery power.

![NEO-M8T_FeatherWing_PowerTop](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/NEO-M8T_FeatherWing_PowerTop.JPG)

![NEO-M8T_FeatherWing_PowerBottom](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/NEO-M8T_FeatherWing_PowerBottom.JPG)

If you want to completely power down the Adalogger _and_ the NEO-M8T, you can do this by pulling the Feather EN pin low (e.g. by connecting it to GND via a small slide switch).

![NEO-M8T_FeatherWing_EN](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/NEO-M8T_FeatherWing_EN.JPG)

![NEO-M8T_FeatherWing_ENswitch](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/NEO-M8T_FeatherWing_ENswitch.JPG)

## Serial Data Pins

![NEO-M8T_FeatherWing_TxRxTop](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/NEO-M8T_FeatherWing_TxRxTop.JPG)

![NEO-M8T_FeatherWing_TxRxBottom](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/NEO-M8T_FeatherWing_TxRxBottom.JPG)

The NEO-M8T FeatherWing, like the Ultimate GPS FeatherWing, can communicate over UART serial. It sends ASCII NMEA sentences from the TX pin to the microcontroller RX pin and can be controlled to change its data output from the microcontroller TX pin. Logic level is 3V for both.

The u-box M8 chipset also supports much more comprehensive UBX binary format messages (see [Serial Protocol](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/LEARN.md#serial-protocol) below).

The baud rate by default is 9600 baud, but you can configure the module to use a different baud rate if desired. For RAWX logging you will need to use at least 115200 baud.

The NEO-M8T Tx and Rx pins are wired directly to the Serial pins at the bottom right of the Feather.
If you need to connect to a different set of pins, you can cut the RX and TX jumpers on the bottom of the board and connect instead to the TX and RX pads at the top of the board.

The ATSAMD21G18 ARM Cortex M0 chip on the Adalogger has multiple Sercom channels which can be used to provide additional serial ports, which is handy if you're already using Serial1 for something else:
- Adafruit: https://learn.adafruit.com/using-atsamd21-sercom-to-add-more-spi-i2c-serial-ports/creating-a-new-serial
- MartinL: https://forum.arduino.cc/index.php?topic=341054.msg2443086#msg2443086

## I2C Interface

![NEO-M8T_FeatherWing_I2CTop](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/NEO-M8T_FeatherWing_I2CTop.JPG)

![NEO-M8T_FeatherWing_I2CBottom](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/NEO-M8T_FeatherWing_I2CBottom.JPG)

The NEO-M8T also provides an I2C (DDC) interface. The SCL and SDA pins are broken out onto two pads near the top right corner of the board. Pull up resistors are provided (R5 & R6).

The SCL and SDA pads are not connected directly to SCL and SDA on the Adalogger. You will need to add your own jumper wires if you do want to make use of this interface.

## SPI Interface

The NEO-M8T SPI interface can be enabled by shorting the DSEL split pad:
- SDA becomes SPI CS
- SCL becomes SPI CLK
- TX becomes SPI MISO
- RX becomes SPI MOSI
- You may need to remove the SCL and SDA pull-up resistors (R5 & R6)
- You will need to cut the TX and RX jumpers to isolate the Adalogger Serial pins

![NEO-M8T_FeatherWing_DSEL](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/NEO-M8T_FeatherWing_DSEL.JPG)

## Reset Button

![NEO-M8T_FeatherWing_ResetTop](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/NEO-M8T_FeatherWing_ResetTop.JPG)

![NEO-M8T_FeatherWing_ResetBottom](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/NEO-M8T_FeatherWing_ResetBottom.JPG)

There is a small button that will connect the microcontroller RESET pin to ground for resetting it. Handy to restart the Feather. Note that this is not connected to NEO-M8T reset unless you short the GRESET split pad on the rear of the PCB (see below).

## GNSS Breakout Pins

![NEO-M8T_FeatherWing_BreakoutPins](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/NEO-M8T_FeatherWing_BreakoutPins.JPG)

**GRST** is connected to the NEO-M8T reset pin. You can reset the NEO-M8T by pulling this pin low. If you want to reset both the Feather _and_ the NEO-M8T via the reset button, short the GRESET split pad on the rear of the PCB.
Note that pulling GPS Reset low does not put the NEO-M8T into a low power state, you'll want to disable it instead (see En below).

![NEO-M8T_FeatherWing_GReset](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/NEO-M8T_FeatherWing_GReset.JPG)

**INT** is connected to the NEO-M8T EXTINT external interrupt pin. It can be used for control of the receiver or for aiding. See the [u-blox documentation links](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/LEARN.md#datasheets) below for further details.

**TP** is connected to the NEO-M8T TP time pulse pin. It can be used to output pulse trains synchronized with GPS or UTC time grid with intervals configurable over a wide frequency range.
Thus it may be used as a low frequency time synchronization pulse or as a high frequency reference signal (up to 10 MHz). By default, the time pulse signal is configured to 1 pulse per second.

TP is also connected to an LED via a buffer transistor. By default it will flash once per second once the receiver is synchronised to GNSS time.

![NEO-M8T_FeatherWing_TPLED](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/NEO-M8T_FeatherWing_TPLED.JPG)

**En** is a true 'power disable' control line you can use to completely cut power to the NEO-M8T. This is good if you need to run at ultra-low-power modes. By default this is pulled low (enabled). So pull high (to 3V) to disable the NEO-M8T.

## SAFEBOOT

![NEO-M8T_FeatherWing_SafeBoot](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/NEO-M8T_FeatherWing_SafeBoot.JPG)

The NEO-M8T SAFEBOOT pin is broken out on a test pad on the rear of the PCB. You will need access to this to update the firmware.

## Battery Backup

Like the Ultimate GPS FeatherWing, the NEO-M8T FeatherWing includes a holder for a CR1220 back-up battery which will keep the NEO-M8T's internal clock going if the power is removed or disabled, providing a much quicker "warm start" when the power is reconnected.

If you don't want to use the backup battery, you can instead draw backup power from the 3V pin by shorting the 3V BACKUP split pad. But be careful! Only do this if you **won't** be using the backup battery.
If you install the battery _and_ have the split pad shorted **BAD THINGS WILL HAPPEN!** You might want to put some tape over the battery holder if you have shorted the split pad.

![NEO-M8T_FeatherWing_BackupTop](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/NEO-M8T_FeatherWing_BackupTop.JPG)

![NEO-M8T_FeatherWing_BackupBottom](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/NEO-M8T_FeatherWing_BackupBottom.JPG)

## Antenna

The NEO-M8T needs to be connected to an external antenna via a uFL connector. 3V power for an active antenna is provided.

![NEO-M8T_FeatherWing_Antenna](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/NEO-M8T_FeatherWing_Antenna.JPG)

## Serial Protocol

The NEO-M8T is based on u-blox's sophisticated M8 chipset. By default it will output standard NMEA format navigation messages, but also supports much more comprehensive UBX binary format messages.
As the NEO-M8T is able to provide location information from GPS, GLONASS, Galileo and BeiDou you will find that the prefix of the NMEA messages changes to: $GP for GPS, SBAS and QZSS; $GL for GLONASS; $GA for Galileo; $GB for BeiDou; or $GN if it is using a mix of satellites (which is usually the case).
Having the messages prefixed with $GN will confuse the Adafruit GPS Library and Mikal Hart's TinyGPS. To get round this, you can force the "Talker ID" to GP by sending the following UBX binary format message:

- 0xb5, 0x62, 0x06, 0x17, 0x14, 0x00, 0x20, 0x40, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x96, 0xd9

The M8 chipset supports several different navigation modes including: static, portable, pedestrian, automotive, sea and "Airborne <1G" (which is really useful for high altitude ballooning!).

By default, the NEO-M8T will receive signals from GPS, QZSS and GLONASS. If you want to enable Galileo and/or BeiDou reception, you'll need to do this with another UBX message (see below).

You can find the messages to change the navigation mode and GNSS configuration in the [RAWX_Logger](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/tree/master/Arduino/RAWX_Logger) Arduino code.

## RAWX_Logger

[RAWX_Logger](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/tree/master/Arduino/RAWX_Logger) is Arduino code written to run on the [Adafruit Feather M0 Adalogger](https://www.adafruit.com/products/2796).
- https://www.adafruit.com/products/2796
- https://learn.adafruit.com/adafruit-feather-m0-adalogger

Rover data is logged to a file called _r_hhmmss.ubx_, base (static) data is logged to a file called _b_hhmmss.ubx_. The code logs RAWX data continuously to SD card until the stop button is pressed.
Since the code is logging a large amount of data, it is necessary to keep the log file open throughout. If the power is removed or the board is reset before the stop button is pressed, the logged data will be lost.

[RAWX_Logger_2](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/tree/master/Arduino/RAWX_Logger_2) is an improved version of the logger.
A new log file is automatically opened every _INTERVAL_ minutes to minimise data loss in the event of a power failure.

Connect a normally-open push-to-close switch between swPin and GND. By default, swPin is Digital Pin 15 (0.2" away from the GND pin on the Adalogger). The pin can be changed by editing the code.

![NEO-M8T_FeatherWing_Adalogger_StopSwitch](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/NEO-M8T_FeatherWing_Adalogger_StopSwitch.JPG)

By default, RAWX data is logged every 250 msec (4Hz). This can be slowed down by selecting an alternate CFG-RATE message.

By default, the code will log raw measurements from GPS + Galileo + GLONASS + SBAS. This can be changed to GPS + Galileo + BeiDou + SBAS by commenting out the line which says _#define GLONASS_

By default, the code will use the static navigation mode. The chosen mobile (roving) mode can be selected by commenting out the line which says _#define STATIC_

The code now calculates the UBX checksum bytes for you, so it is much easier to make changes to the GNSS configuration. See sendUBX() in the code.

You will need to increase the size of the serial buffer to avoid data overruns while data is being written to the SD card. See this post by MartinL:
- https://forum.arduino.cc/index.php?topic=365220.0

For the Adafruit Feather M0 Adalogger (SAMD), under Windows, edit:
- C:\Users\ ...your_user... \AppData\Local\Arduino15\packages\adafruit\hardware\samd\1.0.19\cores\arduino\RingBuffer.h

and change this line:
- #define SERIAL_BUFFER_SIZE 164

For RAWX_Logger, change it to:
- #define SERIAL_BUFFER_SIZE 2048

For RAWX_Logger_2, change it to:
- #define SERIAL_BUFFER_SIZE 8192

Check the reported freeMemory before and after to make sure the change has been successful. (You should find that you've lost twice as much memory as expected!)

The code uses: the Adafruit GPS Library; Bill Greiman's SdFat; and Michael P. Flaga's MemoryFree. See below for the download links.

## PC Logging

To log the RAWX data to file on a PC (instead of the Adalogger SD card):
- The [UBX_Echo](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/tree/master/Arduino/UBX_Echo) directory contains Arduino code for the Adalogger which will change the NEO-M8T Baud rate to 115200 and then echo all data to the PC.
- [NEO-M8T_GNSS_RAWX_Logger.py](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/Python/NEO-M8T_GNSS_RAWX_Logger.py) is Python code which configures the NEO-M8T and then logs the RAWX data to file on a PC.
- [UBX_Checker.py](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/Python/UBX_Checker.py) is Python code which can be used to check the integrity of the RAWX file (to make sure no data has been lost).

Hidden in [NEO-M8T_GNSS_RAWX_Logger.py](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/Python/NEO-M8T_GNSS_RAWX_Logger.py) is code which calculates the UBX message checksums.

## Precise Positioning Resources

Useful precise positioning resources can be found at:
- https://rtklibexplorer.wordpress.com
- http://rtkexplorer.com/
- http://www.rtklib.com
- https://github.com/tomojitakasu/RTKLIB

## Datasheets

Useful documentation about the NEO-M8T and its protocol specification can be found at:
- [NEO-LEA-M8T-FW3_ProductSummary](https://www.u-blox.com/sites/default/files/products/documents/NEO-LEA-M8T-FW3_ProductSummary_%28UBX-16000801%29.pdf)
- [NEO-LEA-M8T-FW3_DataSheet](https://www.u-blox.com/sites/default/files/NEO-LEA-M8T-FW3_DataSheet_%28UBX-15025193%29.pdf)
- [NEO-8Q-NEO-M8-FW3_HardwareIntegrationManual](https://www.u-blox.com/sites/default/files/NEO-8Q-NEO-M8-FW3_HardwareIntegrationManual_%28UBX-15029985%29.pdf)
- [u-blox8-M8_ReceiverDescrProtSpec](https://www.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_%28UBX-13003221%29_Public.pdf)
- [CadSoft-Eagle-Library](https://github.com/u-blox/CadSoft-Eagle-Library.git)

## Adafruit GPS Library for Arduino

- https://github.com/adafruit/Adafruit-GPS-Library/

## Bill Greiman's SdFat

- https://github.com/greiman/SdFat

## Michael P. Flaga's MemoryFree

- https://github.com/mpflaga/Arduino-MemoryFree

## Acknowledgements

The NEO-M8T GNSS FeatherWing is based extensively on an original design by Adafruit Industries:
- https://github.com/adafruit/Adafruit-Ultimate-GPS-FeatherWing-PCB

Distributed under a Creative Commons Attribution, Share-Alike license

Adafruit invests time and resources providing this open source design, please support Adafruit and open-source hardware by purchasing products from Adafruit!

Enjoy!

### _Paul_