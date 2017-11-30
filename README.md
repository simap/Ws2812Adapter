Apa102 Adapter
=========

This is a lightweight buffer-less driver to send pixel data out via UART1 on the ESP8266 to drive ws2812 aka NeoPixel LED strips.

Instead of setting pixels in a buffer then sending that data out, this calls out to a function to generate each pixel. The pixel data could come from some other buffer, or be created on the fly.

This uses less memory and can theoretically drive a very long chain of LEDs.


