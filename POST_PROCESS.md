# Initial attempts at Post-Process Precise Positioning using the NEO-M8T GNSS FeatherWing

## 2017-09-23

Following on from [last week's update](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/POST_PROCESS.md#2017-09-17),
I decided to download [rtkexplorer's demo5 version of RTKLIB](http://rtkexplorer.com/downloads/rtklib-code/) so I could run RTKLIB locally on my machine
rather than pester rtklibexplorer via the [demo service](https://rtklibexplorer.wordpress.com/2017/08/25/online-rtklib-post-processing-demo-service/).
I downloaded version B28a, but I see that version B29 has just been released.

I'm using Windows 10 Pro 64-bit and the executables ran straight out of the box.

To start, I used RTKCONV to convert the raw u-blox files from the base and rover loggers into RINEX format:

![rtkconv.JPG](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/rtkconv.JPG)

I changed the data format setting to _u-blox_, left the options set to their default settings and then converted first the base data (base_20170917_183829.ubx) and then the rover data (rov_20170917_183928.ubx).
This created the .nav and .obs files needed by RTKPOST.

Next I ran RTKPOST:

![rtkpost.JPG](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/rtkpost.JPG)

I changed the options to:

![rtkpost_options_1.JPG](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/rtkpost_options_1.JPG)
![rtkpost_options_2.JPG](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/rtkpost_options_2.JPG)
![rtkpost_options_3.JPG](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/rtkpost_options_3.JPG)

I set:
- the _RINEX OBS: Rover_ file to the _rov .obs_ file created by RTKCONV
- the _RINEX OBS: Base Station_ file to the _base .obs_ file created by RTKCONV
- the _RINEX NAV/CLK_ file to the _rov .nav_ file created by RTKCONV

After pressing _Execute_ and then _Plot_, this is what I ended up with:

![rtkpost_plot_fwd.JPG](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/rtkpost_plot_fwd.JPG)

So far so good. I then changed the Filter Type setting to _Combined_ so the data is processed both forwards and backwards in time:

![rtkpost_options_4.JPG](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/rtkpost_options_4.JPG)

**_Hey presto_** I ended up with this:

![rtkpost_plot_combined.JPG](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/rtkpost_plot_combined.JPG)

Now, that really does look like a very clean 1m radius circle. Perfect!

I'm going to try and do a least-squares fit and analysis on the data with Python, but that will have to wait until later...

## 2017-09-17

It has been a great weekend! After populating the first two NEO-M8T GNSS FeatherWing PCBs last weekend,
this weekend I found time to give them a try and see if they showed any signs of delivering sub-cm accuracy post-processed precise positioning.

**The early signs are very encouraging!**

I'm very grateful to rtklibexplorer for this post:
- https://rtklibexplorer.wordpress.com/2017/08/25/online-rtklib-post-processing-demo-service/  

and for offering his demo service for free. What a guy.

### The Base Logger

The Base Logger is a NEO-M8T GNSS FeatherWing mounted on an Adafruit Feather M0 Adalogger, connected to the GPS-half of a
[Iridium Beam Whip Dual Mode Antenna RST706](https://www.beamcommunications.com/products/70-iridium-beam-whip-dual-mode-antenna)
which is mounted on the study roof with a reasonable view of the sky.

![Post_Process_Test_Base_1.JPG](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/Post_Process_Test_Base_1.JPG)
![Post_Process_Test_Base_2.JPG](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/Post_Process_Test_Base_2.JPG)

The NEO-M8T is set to "Static" navigation mode by the Arduino code. See [LEARN.md](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/LEARN.md) for more details.

### The Rover Logger

The Rover Logger is a second NEO-M8T GNSS FeatherWing mounted on another Adafruit Feather M0 Adalogger, powered by a 1000mAh LiPo battery,
connected to a passive helical antenna **with no ground plane**. The logger assembly is then mounted on the end of a rotating arm whirligig.

![Post_Process_Test_Mobile_2.JPG](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/Post_Process_Test_Mobile_2.JPG)
![Post_Process_Test_Mobile_1.JPG](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/Post_Process_Test_Mobile_1.JPG)

The NEO-M8T is set to "Portable" navigation mode by the Arduino code.

The antenna is positioned at a radius of 1m from the pivot. I think the position is accurate to a mm or so.

The antenna (and logger) rotate at approx. 22 revs per minute, or 2.3 m/s (5.1 mph). There's a video of it doing its thing here:
- https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/Post_Process_Mobile.mpeg

### The Arduino Code

I had a lightbulb moment earlier today when I realised that the loggers need to log RXM-SFRBX and TIM-TM2 messages in addition to RXM-RAWX.
Once I figured that out, everything dropped into place.

The updated [RAWX_Logger](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/tree/master/Arduino/RAWX_Logger) Arduino code logs these messages to SD card four times per second.

### rtklibexplorer's rtklib-post-processing-demo-service

Once I'd collected 15 minutes' worth of data from base and rover, the only thing I needed to do was change the filenames:
- Base:  from 20170917\183829.bin to base_20170917_183829.ubx
- Rover: from 20170917\183928.bin to rov_20170917_183928.ubx

I attached these files to an email, with the subject "rtklib demo", and sent it off to rtklibexplorer as per the instructions in the [blog post](https://rtklibexplorer.wordpress.com/2017/08/25/online-rtklib-post-processing-demo-service/).
Minutes later the analysis came back:

Base Observations:

![plot_obs_base.jpg](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/plot_obs_base.jpg)

Rover Observations:

![plot_obs_rover.jpg](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/plot_obs_rover.jpg)

Position Solution computed with the demo5 B28b version of RTKLIB. Yellow represents a float solution and green is a fixed solution:

![plot_demo5.jpg](https://github.com/PaulZC/NEO-M8T_GNSS_FeatherWing/blob/master/img/plot_demo5.jpg)

Clearly I've got some more work to do, but that certainly looks like a 1m radius circle to me!

The next thing I'll try is adding a ground plane for the rover antenna... But I'll have to save that until next weekend. Back to the day job for now...


Enjoy!

**_Paul_**