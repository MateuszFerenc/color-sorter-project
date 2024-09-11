BIN=main
OBJS=main.o
DIR=Bin

DEVICE=atmega16a
SPEED=14745600UL

OBJCOPY=avr-objcopy
AVRSIZE=avr-size
AVROBJDUMP=avr-objdump

CC=avr-gcc
CFLAGS=-DF_CPU=${SPEED} -mmcu=${DEVICE} -Os -Wfatal-errors -Wall

PROGRAMMER=USBasp
AVRDUDE=avrdude

DEL=del

all: ${OBJS} ${BIN}.elf install

%.elf: %.c
		${CC} ${CFLAGS} $< -o ${DIR}/$@

%.hex: %.elf
		${OBJCOPY} -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures -O ihex ${DIR}/$< ${DIR}/$@
		${AVRSIZE} --mcu=${DEVICE} -C -x ${DIR}/${BIN}.elf
		${AVRSIZE} -B -x ${DIR}/${BIN}.elf --mcu=${DEVICE} -d

eeprom: ${BIN}.elf
		${OBJCOPY} -j .eeprom --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0 -O ihex ${DIR}/$< ${DIR}/${BIN}.eep

debug:
		${AVROBJDUMP} -h -Ss ${DIR}/${BIN}.elf > ${DIR}/${BIN}.lst

install: ${BIN}.hex
		${AVRDUDE} -c ${PROGRAMMER} -p ${DEVICE} -U flash:w:${DIR}/$<

program_eeprom:	${BIN}.eep
		${AVRDUDE} -c ${PROGRAMMER} -p ${DEVICE} -U eeprom:w:${DIR}/$<

clean:
	${DEL} ${DIR}/${BIN}.elf ${DIR}/${BIN}.hex ${DIR}/${BIN}.lst ${DIR}/${OBJS}

# avrdude -c USBasp -p atmega16a	-U lfuse:w:0xFE:m	-U hfuse:w:0xC1:m

# TODO fix directories