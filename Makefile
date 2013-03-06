# fhcore makefile
#
# This makefile allows you to build sketches from the command line
# without the Arduino environment (or Java).
#
#
# $Id$

#
#	From watching the Java IDE do 
#
#-I/Applications/Arduino.app/Contents/Resources/Java/hardware/arduino/cores/arduino
#-I/Applications/Arduino.app/Contents/Resources/Java/hardware/arduino/variants/mega
#
#	ibid
# WInterrupts.c.o
# wiring.c.o
# wiring_analog.c.o
# wiring_digital.c.o
# wiring_pulse.c.o
# wiring_shift.c.o
# CDC.cpp.o
# HardwareSerial.cpp.o
# HID.cpp.o
# IPAddress.cpp.o
# main.cpp.o
# new.cpp.o
# Print.cpp.o
# Stream.cpp.o
# Tone.cpp.o
# USBCore.cpp.o
# WMath.cpp.o
# WString.cpp.o


PORT = /dev/tty.usbmodem*
TARGET = fhcore
ARDUINO = /Applications/Arduino.app/Contents/Resources/Java/hardware/arduino/cores/arduino/
MOREARDUINO = /Applications/Arduino.app/Contents/Resources/Java/hardware/arduino/variants/mega

#SRC = $(ARDUINO)/buffer.c $(ARDUINO)/pins_arduino.c $(ARDUINO)/Serial.c $(ARDUINO)/uart.c $(ARDUINO)/wiring.c
SRC = $(ARDUINO)/WInterrupts.c $(ARDUINO)/wiring.c  $(ARDUINO)/wiring_analog.c $(ARDUINO)/wiring_digital.c \
 	$(ARDUINO)/wiring_pulse.c $(ARDUINO)/wiring_shift.c
CXXSRC = $(TARGET).cpp $(ARDUINO)/HardwareSerial.cpp $(ARDUINO)/Print.cpp  \
	$(ARDUINO)/CDC.cpp $(ARDUINO)/HID.cpp $(ARDUINO)/IPAddress.cpp $(ARDUINO)/main.cpp \
	$(ARDUINO)/new.cpp $(ARDUINO)/Stream.cpp $(ARDUINO)/Tone.cpp $(ARDUINO)/USBCore.cpp \
	$(ARDUINO)/WMath.cpp $(ARDUINO)/WString.cpp
MCU = atmega2560
F_CPU = 16000000
FORMAT = ihex
#UPLOAD_RATE = 19200
UPLOAD_RATE = 115200

# Name of this Makefile (used for "make depend").
MAKEFILE = Makefile

# Debugging format.
# Native formats for AVR-GCC's -g are stabs [default], or dwarf-2.
# AVR (extended) COFF requires stabs, plus an avr-objcopy run.
DEBUG = stabs

OPT = s

# Place -D or -U options here
CDEFS = -DF_CPU=$(F_CPU)
CXXDEFS = -DF_CPU=$(F_CPU)

# Place -I options here
CINCS = -I$(ARDUINO) -I$(MOREARDUINO)
CXXINCS = -I$(ARDUINO)

# Compiler flag to set the C Standard level.
# c89   - "ANSI" C
# gnu89 - c89 plus GCC extensions
# c99   - ISO C99 standard (not yet fully implemented)
# gnu99 - c99 plus GCC extensions
CSTANDARD = -std=gnu99
CDEBUG = -g$(DEBUG)
CWARN = -Wall -Wstrict-prototypes
CTUNING = -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
#CEXTRA = -Wa,-adhlns=$(<:.c=.lst)

CFLAGS = $(CDEBUG) $(CDEFS) $(CINCS) -O$(OPT) $(CWARN) $(CSTANDARD) $(CEXTRA)
CXXFLAGS = $(CDEFS) $(CINCS) -O$(OPT)
#ASFLAGS = -Wa,-adhlns=$(<:.S=.lst),-gstabs 
LDFLAGS = 


# Programming support using avrdude. Settings and variables.
AVRDUDE_PROGRAMMER = wiring
AVRDUDE_PORT = $(PORT)
AVRDUDE_WRITE_FLASH = -U flash:w:$(TARGET).hex
AVRDUDE_FLAGS = -F -p $(MCU) -P $(AVRDUDE_PORT) -c $(AVRDUDE_PROGRAMMER) \
  -b $(UPLOAD_RATE)

# Program settings
PROGLOCATION = /Applications/Arduino.app/Contents/Resources/Java/hardware/tools/avr/bin
CC = $(PROGLOCATION)/avr-gcc
CXX = $(PROGLOCATION)/avr-g++
OBJCOPY = $(PROGLOCATION)/avr-objcopy
OBJDUMP = $(PROGLOCATION)/avr-objdump
SIZE = avr-size
NM = avr-nm
#AVRDUDE = avrdude
AVRDUDE = /Applications/Arduino.app/Contents/Resources/Java/hardware/tools/avr/bin/avrdude -v -v -v -v -C/Applications/Arduino.app/Contents/Resources/Java/hardware/tools/avr/etc/avrdude.conf
REMOVE = rm -f
MV = mv -f

# Define all object files.
OBJ = $(SRC:.c=.o) $(CXXSRC:.cpp=.o) $(ASRC:.S=.o)

# Define all listing files.
LST = $(ASRC:.S=.lst) $(CXXSRC:.cpp=.lst) $(SRC:.c=.lst)

# Combine all necessary flags and optional flags.
# Add target processor to flags.
ALL_CFLAGS = -mmcu=$(MCU) -I. $(CFLAGS)
ALL_CXXFLAGS = -mmcu=$(MCU) -I. $(CXXFLAGS)
ALL_ASFLAGS = -mmcu=$(MCU) -I. -x assembler-with-cpp $(ASFLAGS)


# Default target.
all: build

build: elf hex eep

elf: $(TARGET).elf
hex: $(TARGET).hex
eep: $(TARGET).eep
lss: $(TARGET).lss 
sym: $(TARGET).sym

# Program the device.  
upload: $(TARGET).hex $(TARGET).eep
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH)




# Convert ELF to COFF for use in debugging / simulating in AVR Studio or VMLAB.
COFFCONVERT=$(OBJCOPY) --debugging \
--change-section-address .data-0x800000 \
--change-section-address .bss-0x800000 \
--change-section-address .noinit-0x800000 \
--change-section-address .eeprom-0x810000 


coff: $(TARGET).elf
	$(COFFCONVERT) -O coff-avr $(TARGET).elf $(TARGET).cof


extcoff: $(TARGET).elf
	$(COFFCONVERT) -O coff-ext-avr $(TARGET).elf $(TARGET).cof


.SUFFIXES: .elf .hex .eep .lss .sym

.elf.hex:
	$(OBJCOPY) -O $(FORMAT) -R .eeprom $< $@

.elf.eep:
	-$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	--change-section-lma .eeprom=0 -O $(FORMAT) $< $@

# Create extended listing file from ELF output file.
.elf.lss:
	$(OBJDUMP) -h -S $< > $@

# Create a symbol table from ELF output file.
.elf.sym:
	$(NM) -n $< > $@



# Link: create ELF output file from object files.
$(TARGET).elf: $(OBJ)
	$(CC) $(ALL_CFLAGS) $(OBJ) --output $@ $(LDFLAGS)


# Compile: create object files from C++ source files.
.cpp.o:
	$(CXX) -c $(ALL_CXXFLAGS) $< -o $@ 

# Compile: create object files from C source files.
.c.o:
	$(CC) -c $(ALL_CFLAGS) $< -o $@ 


# Compile: create assembler files from C source files.
.c.s:
	$(CC) -S $(ALL_CFLAGS) $< -o $@


# Assemble: create object files from assembler source files.
.S.o:
	$(CC) -c $(ALL_ASFLAGS) $< -o $@



# Target: clean project.
clean:
	$(REMOVE) $(TARGET).hex $(TARGET).eep $(TARGET).cof $(TARGET).elf \
	$(TARGET).map $(TARGET).sym $(TARGET).lss \
	$(OBJ) $(LST) $(SRC:.c=.s) $(SRC:.c=.d) $(CXXSRC:.cpp=.s) $(CXXSRC:.cpp=.d)

depend:
	if grep '^# DO NOT DELETE' $(MAKEFILE) >/dev/null; \
	then \
		sed -e '/^# DO NOT DELETE/,$$d' $(MAKEFILE) > \
			$(MAKEFILE).$$$$ && \
		$(MV) $(MAKEFILE).$$$$ $(MAKEFILE); \
	fi
	echo '# DO NOT DELETE THIS LINE -- make depend depends on it.' \
		>> $(MAKEFILE); \
	$(CC) -M -mmcu=$(MCU) $(CDEFS) $(CINCS) $(SRC) $(ASRC) >> $(MAKEFILE)

.PHONY:	all build elf hex eep lss sym program coff extcoff clean depend