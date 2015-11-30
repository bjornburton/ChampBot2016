# Project Name
TARGET:= jafco

# Which target board
BOARD:=trinket

# Which microcontroller
MCU:=attiny85

# Which avrdude to use
PROGSW:=avrdude

# Which fuses
LFUSE:= lfuse:w:0x62:m //default is 62, Trinket like F2
HFUSE:= hfuse:w:0xDF:m 
EFUSE:= efuse:w:0xFF:m


# Apps and Flags
PROGSWFLAGS:= -p $(BOARD)
CC       := avr-gcc 
DOT       := dot 
EGYPT     := egypt 
TANGLE   := ctangle
WEAVE    := cweave
TEX      := pdftex
CFLAGS   :=  -std=c99 -g -mmcu=$(MCU) -Wall -Os -pedantic -fdump-rtl-expand
CONV     :=avr-objcopy
CONVFLAGS:= -j .text -j .data -O ihex
LIBS     :=
DOTFLAGS  := -Tpng

# Build filenames
HEADERS := $(TARGET).h
OBJECTS := $(TARGET).o
HEX     := $(TARGET).hex
ELF     := $(TARGET).elf
CSOURCES:= $(TARGET).c
WEB     := $(TARGET).w
DOC     := $(TARGET).pdf
TEXSRC  := $(TARGET).tex
DOTSRC   := $(TARGET).dot
EGYPTSRC := $(TARGET).c.*r.expand
GRAPH    := $(TARGET).png

# The usual make stuff
default: $(HEX)
elf: $(ELF)
all: default

$(GRAPH): $(DOTSRC)
	$(DOT) $(DOTFLAGS) $(DOTSRC) > $(GRAPH)

$(DOTSRC): $(EGYPTSRC)
	$(EGYPT) $(EGYPTSRC) |\
        awk '{gsub(/__vector_2/,"PCINT0_vect");print}' \
         > $(DOTSRC)


$(EGYPTSRC): $(OBJECTS)
$(CSOURCES): $(WEB) $(GRAPH)
	$(TANGLE) $(WEB)
	$(WEAVE) $(WEB)
	$(TEX) $(TEXSRC) 

$(OBJECTS): $(CSOURCES)
	$(CC) -c $(CFLAGS) $(CSOURCES)

$(ELF): $(OBJECTS)
	$(CC) $(LIBS) $(OBJECTS) $(CFLAGS) -o $(ELF)
	chmod -x $(ELF)

$(HEX): $(ELF)
	$(CONV) $(CONVFLAGS) $(ELF) $(HEX) 

clean:
	-rm -f $(OBJECTS)
	-rm -f $(ELF)
	-rm -f $(TEXSRC)
	-rm -f $(CSOURCES)
	-rm -f $(CSOURCES)
	-rm -f $(DOTSRC)
	-rm -f $(EGYPTSRC)
	
install:
	$(PROGSW) $(PROGSWFLAGS) -c usbtiny -U flash:w:$(HEX)

installasp:
	$(PROGSW) -p $(MCU) -c usbasp -U flash:w:$(HEX)

size:
	avr-size --format=avr --mcu=$(MCU) $(ELF)

fuse:
	$(PROGSW) $(PROGSWFLAGS) -u -U $(LFUSE)  -U $(HFUSE) -U $(EFUSE)
 
