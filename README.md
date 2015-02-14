# stm32f3-minimal-cdc

[![Build Status](https://travis-ci.org/ekiwi/stm32f3-minimal-cdc.svg?branch=master)](https://travis-ci.org/ekiwi/stm32f3-minimal-cdc)

This is a weekend project that tries to allow serial communication
with an STM32F3 discovery board via USB.

A lot of this was written by using Wireshark to see what kinds of requests
the Linux 3.17 kernel sends and then to implement those. So this is
**not** a universal solution.

As I said, this is a weekend project (ok, I started Thursday night), the goal
of which was to have a cdc loopback device working. In fact this is some of
the worst code that I have written. I intentionally avoid the word
`USB stack` since this, in no way, is a stack. It is just a bunch of
spaghetti code that kinda works ... in one configuration.

It's a long way until this might make it into
[xpcc](https://github.com/roboterclubaachen/xpcc).
But if you feel a little bit adventurous you should try it out and
report your findings on different platforms. Please attach a
[Wireshark USB capture file](http://wiki.wireshark.org/CaptureSetup/USB)
to your bug report.

If you are still thinking about using this in any "real" product,
please note, that I have not read the USB specification. So please
take this part of the GPLv3 seriously:

~~~
minmal-cdc is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE
~~~

## TODO
* somehow it seems to take way too long (several seconds) until one
  open the /dev/ttyACMx device, but this seems to be similar with the
  teensy 3.1
