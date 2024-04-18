BIN=main
OBJS=main.o

DEVICE=atmega48
SPEED=1000000UL

CC="C:\Users\mateo\Desktop\Projects\C\AVR PROGRAMMING\avr8-gnu-toolchain-win32_x86_64\bin\avr-gcc"
OBJCOPY="C:\Users\mateo\Desktop\Projects\C\AVR PROGRAMMING\avr8-gnu-toolchain-win32_x86_64\bin\avr-objcopy"
AVRSIZE="C:\Users\mateo\Desktop\Projects\C\AVR PROGRAMMING\avr8-gnu-toolchain-win32_x86_64\bin\avr-size"
AVROBJDUMP="C:\Users\mateo\Desktop\Projects\C\AVR PROGRAMMING\avr8-gnu-toolchain-win32_x86_64\bin\avr-objdump"
CFLAGS=-Os -DF_CPU=${SPEED} -mmcu=${DEVICE} -Wall
PROGRAMMER=USBasp
AVRDUDE="C:\Users\mateo\Desktop\Projects\C\AVR PROGRAMMING\avrdude-6.4-mingw32\avrdude"

all: compile objcopy install

compile: ${OBJS}
		${CC} ${CFLAGS} -g -o ${BIN}.elf $^

objcopy: ${BIN}.elf
		${OBJCOPY} -j .text -j .data -O ihex $< ${BIN}.hex
		${AVRSIZE} ${BIN}.elf

#debug:
#		${AVROBJDUMP} -h -S ${BIN}.elf > ${BIN}.lst

install: ${BIN}.hex
	${AVRDUDE} -c ${PROGRAMMER} -p ${DEVICE} -U flash:w:$<

clean:
	del ${BIN}.elf ${BIN}.hex ${OBJS}