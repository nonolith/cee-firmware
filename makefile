DEVICE  = atxmega32a4
F_CPU   = 32000000UL
F_USB	= 48000000UL
CFLAGS  =  -I. -Iusb/ -include usb/io.h -std=c99

OBJECTS = Descriptors.o  main.o  packetbuffer.o usb/usb_requests.o  usb/usb_xmega.o

USB_FLAGS =  -DF_USB=$(F_USB) -D USB_MAXEP=2 -D USB_EP0SIZE=64

COMPILE = avr-gcc -Wall -O3 -DF_CPU=$(F_CPU) $(CFLAGS) -mmcu=$(DEVICE) $(USB_FLAGS)

hex: main.hex

.c.o:
	$(COMPILE) -c $< -o $@

clean:
	rm -f */*.o *.o *.hex *.elf

main.elf: $(OBJECTS)
	$(COMPILE) -o main.elf $(OBJECTS)

main.hex: main.elf
	rm -f main.hex
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex
	avr-size main.hex

c:
	$(COMPILE) -E usb/usb_xmega.c
	$(COMPILE) -E usb/usb_requests.c 
	$(COMPILE) -E main.c 
	$(COMPILE) -E Descriptors.c
	$(COMPILE) -E packetbuffer.o

update: main.hex
	python update.py main.hex

HW_PRODUCT=Nonolith CEE
HW_VERSION=1.0
FW_VERSION=$(shell git describe --always --dirty='*')

bootload_only:
	make -Cusb/bootloader HW_PRODUCT="$(HW_PRODUCT)" HW_VERSION="$(HW_VERSION)" program
	sleep 0.5

cee.json: cee.hex
	python make_fwupdate.py cee.hex "$(FW_VERSION)"
