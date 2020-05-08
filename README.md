Ws2812 Adapter
=========

This is a lightweight buffer-less driver to send pixel data out via UART1 on the ESP8266 to drive ws2812/sk6812 aka NeoPixel LED strips.

Instead of setting pixels in a buffer then sending that data out, this calls out to a function to generate each pixel. The pixel data could come from some other buffer, or be created on the fly.

This uses less memory and can theoretically drive a very long chain of LEDs.

A buffered option is also supported.


ESP32
==========

NOTE: this does not use the rmt peripheral. It seems that between the interrupt necessary to fill the buffer and things like async web server (even pinned to another core), buffer underflows are happening and causing glitches.
With async pinned to core0 and this running in core1, no glitches happen.


