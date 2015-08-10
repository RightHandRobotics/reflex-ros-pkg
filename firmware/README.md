Flashing firmware
======

First, you need to get the cross-compiler (gcc-arm) and build OpenOCD, the JTAG transport, and the DFU (Device Firmware Updater) utility. On Ubuntu 12.04 and 14.04 (and potentially others), this is all scripted for you:

    cd firmware/tools
    make
    make dfu

Now you can build the firmware image:

    cd firmware/reflex-takktile
    make clean
    make

That will produce a flash image in the firmware/reflex-takktile/bin directory. To program the microcontroller's flash memory with this image, we can use the microcontroller's built-in ROM bootloader, which will let us connect to the chip directly over USB and program its flash. The STM32F4 processor chooses whether it should boot to the ROM bootloader or the flash-memory program based on whether it's BOOT pin is high or low when its RESET line is released. On the controller board, you will find two buttons. The one nearest the corner of the board is connected to RESET. The one a little further from the corner is the BOOT select line. The neat thing about this setup is that it's impossible to "brick" the chip, even without a JTAG dongle: you can *always* put the chip back in bootloader mode and re-image it. To do this, follow these steps:

1. push and hold the RESET button (the one nearest the corner)
1. push and hold the BOOT button (the other button)
1. release the RESET button (the one nearest the corner)
1. release the BOOT button (the other button)

The LEDs on the controller board will stay dark, and if you have connected your machine to the controller board with a micro-USB cable, you will see that a new USB device has appeared on your system, with VID/PID 0483:df11. You can then issue this command to program the flash image:

    make dfu_download

Then, just press and release the RESET button, and the controller will start executing the application flash image. Hooray!


YOU'RE DONE. FOR DEBUGGING, KEEP READING:
===============================================================

The DFU system is really great for programming flash images and is sufficient if you're doing fairly minor tweaks that don't have a lot of side-effects, but if you need to do full-blown debugging, you'll want to use JTAG. The cheapest way to do this is to buy an Olimex ARM-USB-TINY-H JTAG adapter and their ARM-USB-20-10 pinout converter (since the controller board uses the "new" 50-mil ARM JTAG connector standard). Then, you can connect your USB-JTAG adapter to the controller via JTAG. To program it with a JTAG adapter attached:

    make program

If you want to debug using GDB, you can do this in one terminal to halt the processor and attach to it via JTAG:

    make gdb_server

Then, in another terminal, you can do this to fire up GDB and connect:

    make gdb

Then everything works like in GDB as usual: you can set breakpoints, peek/poke memory, etc. etc.

Hooray.
