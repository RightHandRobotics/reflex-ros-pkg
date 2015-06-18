default: gccarm.installed openocd dfu

# potentially this could be smarter if it detects the OS/distro or whatever.
dfu:
	sudo apt-get install dfu-util

gccarm.installed:
	sudo add-apt-repository ppa:terry.guo/gcc-arm-embedded
	sudo apt-get update
	sudo apt-get install gcc-arm-none-eabi
	touch gccarm.installed

openocd-source:
	# git clone git://git.code.sf.net/p/openocd/code openocd-source
	git clone https://github.com/ntfreak/openocd.git openocd-source

openocd: openocd-source
	sudo apt-get install automake texinfo libusb-1.0-0-dev texi2html
	cd openocd-source && ./bootstrap && ./configure --prefix=$(shell pwd)/openocd --enable-ftdi && make && sudo make install
