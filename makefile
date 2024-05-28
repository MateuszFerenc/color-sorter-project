BIN=main
OBJS=main.o

DEVICE=atmega16
SPEED=14745600UL
#SPEED=8000000UL

OBJCOPY=avr-objcopy
AVRSIZE=avr-size
AVROBJDUMP=avr-objdump

CC=avr-gcc
CFLAGS=-Os -DF_CPU=${SPEED} -mmcu=${DEVICE} #-Wall

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

# avrdude -c USBasp -p atmega16a	-U lfuse:w:0xFE:m	-U hfuse:w:0x89:m