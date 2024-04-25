BIN=main
OBJS=main.o

DEVICE=atmega48
SPEED=1000000UL

OBJCOPY=avr-objcopy
AVRSIZE=avr-size
AVROBJDUMP=avr-objdump

CC=avr-gcc
CFLAGS=-Os -DF_CPU=${SPEED} -mmcu=${DEVICE} -Wall

PROGRAMMER=USBasp
AVRDUDE=avrdude

DEL=del

all: ${OBJS} ${BIN}.elf install

%.elf: %.c
		${CC} ${CFLAGS} $< -o $@

%.hex: %.elf
		${OBJCOPY} -j .text -j .data -O ihex $< $@
		${AVRSIZE} --mcu=${DEVICE} -C -x ${BIN}.elf
		${AVRSIZE} -B -x ${BIN}.elf --mcu=${DEVICE} -d

#debug:
#		${AVROBJDUMP} -h -S ${BIN}.elf > ${BIN}.lst

install: ${BIN}.hex
	${AVRDUDE} -c ${PROGRAMMER} -p ${DEVICE} -U flash:w:$<

clean:
	${DEL} ${BIN}.elf ${BIN}.hex ${OBJS}