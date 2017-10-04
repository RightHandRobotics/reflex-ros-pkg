#!/bin/bash

avrdude -c avrispmkII -p atmega328p -U lfuse:w:0xFF:m
avrdude -c avrispmkII -p atmega328p -U hfuse:w:0xD9:m
avrdude -c avrispmkII -p atmega328p -U efuse:w:0x07:m
avrdude -c avrispmkII -p atmega328p -U flash:w:"./SF_Encoder_Firmware.hex":i
