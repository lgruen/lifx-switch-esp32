# LIFX light switch for ESP32

Implements a simple light switch for an LIFX bulb.

This is not, in any way, affiliated or related to LiFi Labs, Inc.  Use it at
your own risk.

Senses when a capacative button is pressed and then sends a UDP packet over the
WiFi network to change the power state of a particular bulb.

The capacative button can be as simple as a piece of metal connected to one of
the "touch" pins of the ESP32.

This is structured identically to the ESP-IDF examples. Before compiling, run
`make menuconfig` to set up the following:
* "Serial flasher config": The serial port to use for flashing.
* "WiFi Configuration": The WiFi SSID and password.
* "LIFX Configuration": The MAC address of the light bulb that should be
  toggled. It corresponds to the serial number of the bulb
  (see https://support.lifx.com/hc/en-us/articles/205947550-Will-LIFX-work-with-MAC-filtering-).

Then run `make flash` to install to the ESP32. Tested with an ESP32 DevKit V4
board.

Based on two examples that come with ESP-IDF
(see https://docs.espressif.com/projects/esp-idf/en/latest/get-started/):
* `examples/wifi/getting_started/station`
* `examples/peripherals/touch_pad_interrupt`

All the LIFX network-related implementation is copied from
https://github.com/ArquintL/lifx-c-lib, with very minor adjustments to compile
on ESP32.
