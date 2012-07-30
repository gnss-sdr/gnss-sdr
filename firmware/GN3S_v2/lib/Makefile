INCLUDES=-I../include

CC=sdcc -mmcs51 --no-xinit-opt

all: delay.c fx2utils.c isr.c timer.c usb_common.c
	$(CC) $(INCLUDES)  -c delay.c -o delay.rel
	$(CC) $(INCLUDES)  -c fx2utils.c -o fx2utils.rel
	$(CC) $(INCLUDES)  -c isr.c -o isr.rel
	$(CC) $(INCLUDES)  -c timer.c -o timer.rel
	$(CC) $(INCLUDES)  -c usb_common.c -o usb_common.rel
clean: 
	rm -f *.ihx *.rel *.rst *.lnk *.lst *.map *.asm *.sym 
