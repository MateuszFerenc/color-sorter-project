BIN=main
OBJS=main.o
DIR=Bin

DEVICE=atmega16a
SPEED=14745600

OBJCOPY=avr-objcopy
AVRSIZE=avr-size
AVROBJDUMP=avr-objdump

CC=avr-gcc
CFLAGS += -DF_CPU=${SPEED}UL
CFLAGS += -mmcu=${DEVICE}
CFLAGS += -Os
CFLAGS += -Wfatal-errors
CFLAGS += -Wall
CFLAGS += -funsigned-char
CFLAGS += -funsigned-bitfields
CFLAGS += -fpack-struct
CFLAGS += -fshort-enums
CFLAGS += -ffunction-sections
CFLAGS += -fdata-sections
CFLAGS += -Wl,--gc-sections

PROGRAMMER = USBasp
AVRDUDE = avrdude
AVRDUDE_FLAGS = -c ${PROGRAMMER} -p ${DEVICE}

DEL=del

all: hex install

hex: ${BIN}.elf

%.elf: %.c
		${CC} ${CFLAGS} $< -o ${DIR}/$@ 2>${DIR}/compilation_output.txt

%.hex: %.elf
		${OBJCOPY} -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures -O ihex ${DIR}/$< ${DIR}/$@
		${AVRSIZE} --mcu=${DEVICE} -C -x ${DIR}/${BIN}.elf
		${AVRSIZE} -B -x ${DIR}/${BIN}.elf --mcu=${DEVICE} -d

eeprom: ${BIN}.elf
		${OBJCOPY} -j .eeprom --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0 -O ihex ${DIR}/$< ${DIR}/${BIN}.eep

debug:
		${AVROBJDUMP} -h -Ss ${DIR}/${BIN}.elf > ${DIR}/${BIN}.lst

install: ${BIN}.hex
		${AVRDUDE} ${AVRDUDE_FLAGS} -U flash:w:${DIR}/$<

program_eeprom:	${BIN}.eep
		${AVRDUDE} ${AVRDUDE_FLAGS} -U eeprom:w:${DIR}/$<

clean:
	${DEL} "${DIR}\${BIN}.elf" "${DIR}\${BIN}.hex" "${DIR}\${BIN}.lst" "${DIR}\${OBJS}" "${DIR}\${BIN}.eep" "${DIR}\compilation_output.txt"


.DEFAULTGOAL: all
.PHONY: all elf hex eeprom debug install program_eeprom clean

# avrdude -c USBasp -p atmega16a	-U lfuse:w:0xFE:m	-U hfuse:w:0xC1:m