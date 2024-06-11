BIN=main
OBJS=main.o

DEVICE=atmega16
SPEED=14745600UL

OBJCOPY=avr-objcopy
AVRSIZE=avr-size
AVROBJDUMP=avr-objdump

CC=avr-gcc
CFLAGS=-DF_CPU=${SPEED} -mmcu=${DEVICE} -Wall -Os
#-Os

PROGRAMMER=USBasp
AVRDUDE=avrdude

DEL=del

all: ${OBJS} ${BIN}.elf install

%.elf: %.c
		${CC} ${CFLAGS} $< -o $@

%.hex: %.elf
		${OBJCOPY} -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures -O ihex $< $@
		${AVRSIZE} --mcu=${DEVICE} -C -x ${BIN}.elf
		${AVRSIZE} -B -x ${BIN}.elf --mcu=${DEVICE} -d

debug:
		${AVROBJDUMP} -h -Ss ${BIN}.elf > ${BIN}.lst

install: ${BIN}.hex
	${AVRDUDE} -c ${PROGRAMMER} -p ${DEVICE} -U flash:w:$<

clean:
	${DEL} ${BIN}.elf ${BIN}.hex ${BIN}.lst ${OBJS}

# avrdude -c USBasp -p atmega16a	-U lfuse:w:0xFE:m	-U hfuse:w:0x89:m