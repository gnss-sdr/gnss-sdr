INCLUDES=-I../include
CC=sdcc -mmcs51 --no-xinit-opt

all: 	firmware install clean

firmware: gn3s_main.c usrp_common.c init_gpif.c gn3s_gpif.c gn3s_se4120.c
	$(CC) $(INCLUDES) -c gn3s_main.c -o gn3s_main.rel
	$(CC) $(INCLUDES) -c usrp_common.c -o usrp_common.rel
	$(CC) $(INCLUDES) -c init_gpif.c -o init_gpif.rel
	$(CC) $(INCLUDES) -c gn3s_gpif.c -o gn3s_gpif.rel
	$(CC) $(INCLUDES) -c gn3s_se4120.c -o gn3s_se4110.rel
	$(CC) $(INCLUDES) -c eeprom.c -o eeprom.rel
	asx8051 -plosgff vectors.a51
	asx8051 -plosgff usb_descriptors.a51
	asx8051 -plosgff _startup.a51
	$(CC) $(INCLUDES) --code-loc 0x0000 --code-size 0x1800 --xram-loc 0x1800 --xram-size 0x0800 -Wl '-b USBDESCSEG = 0xE000' -L ../lib libfx2.lib -o gn3s_firmware.ihx vectors.rel gn3s_main.rel usrp_common.rel init_gpif.rel gn3s_gpif.rel gn3s_se4110.rel eeprom.rel usb_descriptors.rel _startup.rel

install:
	cp gn3s_firmware.ihx ../bin/
	@rm gn3s_firmware.ihx

clean: 
	rm -f *.ihx *.rel *.rst *.lnk *.lst *.map *.asm *.sym *.mem
