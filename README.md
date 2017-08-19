# NEO-M8T GNSS FeatherWing

A clone of the Adafruit Ultimate GPS FeatherWing but with the u-blox NEO-M8T (external antenna) replacing the GlobalTop FGPMMOPA6H

Designed to work with the Adafruit Feather M0 Adalogger for logging of RAWX GNSS data direct to SD card for post-processed precise positioning using (e.g.) RTKLIB

**19-08-2017: Work in progress... PCB is ready to send for manufacture. Draft Arduino code is complete**

![PCB_TOP](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/PCB_TOP.jpg)

![PCB_BOTTOM](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/PCB_BOTTOM.jpg)

PCB images created using http://viewer.tracespace.io/

See [NEO-M8T_GNSS_FeatherWing.pdf](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/NEO-M8T_GNSS_FeatherWing.pdf) for the BOM etc.

The [Eagle](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/tree/master/Eagle) directory contains the schematic and pcb design

The [RAWX_Logger](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/tree/master/Arduino/RAWX_Logger) directory contains Arduino code which _should_ log the RAWX data to the Adalogger SD card. Works with NAV-PVT messages on the MAX-M8Q, but is currently untested with RXM-RAWX messages on the NEO-M8T

To log the data on a PC:
- The [UBX_Echo](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/tree/master/Arduino/UBX_Echo) directory contains Arduino code to change the NEO-M8T Baud rate to 115200 and then echo all data to the PC
- [NEO-M8T_GNSS_RAWX_Logger.py](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/Python/NEO-M8T_GNSS_RAWX_Logger.py) is Python code which _should_ log the RAWX data to file on a PC. Works with NAV-PVT messages on the SAM-M8Q, but is currently untested with RXM-RAWX messages on the NEO-M8T

Useful precise positioning resources can be found at:
- http://www.rtklib.com
- https://github.com/tomojitakasu/RTKLIB
- https://rtklibexplorer.wordpress.com

Useful documentation about the NEO-M8T and its protocol specification can be found at:
- [NEO-LEA-M8T-FW3_ProductSummary](https://www.u-blox.com/sites/default/files/products/documents/NEO-LEA-M8T-FW3_ProductSummary_%28UBX-16000801%29.pdf)
- [NEO-LEA-M8T-FW3_DataSheet](https://www.u-blox.com/sites/default/files/NEO-LEA-M8T-FW3_DataSheet_%28UBX-15025193%29.pdf)
- [NEO-8Q-NEO-M8-FW3_HardwareIntegrationManual](https://www.u-blox.com/sites/default/files/NEO-8Q-NEO-M8-FW3_HardwareIntegrationManual_%28UBX-15029985%29.pdf)
- [u-blox8-M8_ReceiverDescrProtSpec](https://www.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_%28UBX-13003221%29_Public.pdf)
- [CadSoft-Eagle-Library](https://github.com/u-blox/CadSoft-Eagle-Library.git)

Based on an original design by Adafruit Industries:
- https://www.adafruit.com/product/3133
- https://learn.adafruit.com/adafruit-ultimate-gps-featherwing
- https://github.com/adafruit/Adafruit-Ultimate-GPS-FeatherWing-PCB

Distributed under a Creative Commons Attribution, Share-Alike license

Adafruit invests time and resources providing this open source design, please support Adafruit and open-source hardware by purchasing products from Adafruit!

Enjoy!

# **_Paul_**