SRC=bndrtc
MCU = attiny84


compile: $(SRC).hex

$(SRC).hex: $(SRC).elf
	avr-objcopy -O ihex -j .data -j.text $< $@

$(SRC).elf: $(SRC).o
	avr-gcc -mmcu=$(MCU) $< -o $@

.c.o:
	avr-gcc -c -Os -mmcu=$(MCU) $< -o $@
clean:
	rm -f *.o
	rm -f *.elf
	rm -f *.hex

program:
	avrdude -c avrispMKII -p $(MCU) -Uflash:w:$(SRC).hex:i 
