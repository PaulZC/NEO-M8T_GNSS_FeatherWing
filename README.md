# NEO-M8T GNSS FeatherWing

A clone of the [Adafruit Ultimate GPS FeatherWing](https://www.adafruit.com/product/3133) but with the u-blox NEO-M8T replacing the GlobalTop FGPMMOPA6H. The NEO-M8T can receive signals from GPS, Galileo and GLONASS or BeiDou concurrently and supports both SBAS and QZSS.
It also provides Multi-GNSS Raw Measurement (RAWX) Data which can be used for post-process precise positioning.

![NEO-M8T_FeatherWing](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/NEO-M8T_FeatherWing.JPG)

**See [LEARN.md](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/LEARN.md) for more details.**

[POST_PROCESS.md](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/POST_PROCESS.md) contains results of a first time successful attempt at post-process precise positioning using [rtkexplorer's demo5 version of RTKLIB](http://rtkexplorer.com/downloads/rtklib-code/).

See [NEO-M8T_GNSS_FeatherWing.pdf](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/NEO-M8T_GNSS_FeatherWing.pdf) for the schematic, layout and Bill Of Materials.

The [Eagle](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/tree/master/Eagle) directory contains the schematic and pcb design files.

The [RAWX_Logger](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/tree/master/Arduino/RAWX_Logger) directory contains code for the [Adafruit Feather M0 Adalogger](https://www.adafruit.com/products/2796) which will log RAWX data to SD card.

Distributed under a Creative Commons Attribution, Share-Alike license.

Enjoy!

**_Paul_**