#!/bin/bash

avrdude -p atmega328p -c avrispmkII -P usb -b 57142 -U lfuse:w:0xFF:m
avrdude -p atmega328p -c avrispmkII -P usb -b 57142 -U hfuse:w:0xD9:m
avrdude -p atmega328p -c avrispmkII -P usb -b 57142 -U efuse:w:0xFF:m
avrdude -p atmega328p -c avrispmkII -P usb -b 57142 -U flash:w:"./SF_Encoder_Firmware.hex":i
