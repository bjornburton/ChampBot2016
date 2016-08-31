# Project Name
TARGET:= piruett

# Which target board
# for Trinket use BOARD:=trinket
# for Pro Trinket use 
BOARD:=atmega328p

# Which microcontroller
MCU:=atmega328p

# Which avrdude to use
PROGSW:=avrdude

# Which fuses
LFUSE:= lfuse:w:0xFF:m
HFUSE:= hfuse:w:0xD0:m 
EFUSE:= efuse:w:0x05:m


# Apps and Flags
PROGSWFLAGS:= -p $(BOARD)
CC        := avr-gcc 
DOT       := dot 
EGYPT     := egypt 
TANGLE    := ctangle
WEAVE     := cweave
TEX       := pdftex
CFLAGS    :=  -std=c11 -g -mmcu=$(MCU) -Wall -Os -pedantic -fdump-rtl-expand
CONV      :=avr-objcopy
CONVFLAGS := -j .text -j .data -O ihex
LIBS      :=
DOTFLAGS  := -Tpng



# Build filenames
HEADERS  := $(TARGET).h
OBJECTS  := $(TARGET).o
HEX      := $(TARGET).hex
ELF      := $(TARGET).elf
CSOURCES := $(TARGET).c
WEB      := $(TARGET).w
DOC      := $(TARGET).pdf
TEXSRC   := $(TARGET).tex
DOTSRC   := $(TARGET).dot
EGYPTSRC := $(TARGET).c.*r.expand
GRAPH    := $(TARGET).png


# The usual make stuff
default: $(HEX)
elf:     $(ELF)
all:     default

$(GRAPH): $(DOTSRC)
	$(DOT) $(DOTFLAGS) $(DOTSRC) > $(GRAPH)

$(DOTSRC): $(EGYPTSRC)
	$(EGYPT) $(EGYPTSRC) |\
        awk '{gsub(/__vector_21/,"ADC_vect");print}' |\
        awk '{gsub(/__vector_10/,"TIMER1_CAPT_vect");print}' |\
        awk '{gsub(/__vector_6/,"WDT_vect");print}' > $(DOTSRC)

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
	-rm -f $(DOTSRC)
	-rm -f $(EGYPTSRC)
	
install:
	$(PROGSW) $(PROGSWFLAGS) -c usbtiny -U flash:w:$(HEX)

installasp:
	$(PROGSW) $(PROGSWFLAGS) -c usbasp -U flash:w:$(HEX)

installavr:
	$(PROGSW) $(PROGSWFLAGS) -c avrispmkII -U flash:w:$(HEX)




size:
	avr-size --format=avr --mcu=$(MCU) $(ELF)

fuse:
	$(PROGSW) $(PROGSWFLAGS) -u -U $(LFUSE)  -U $(HFUSE) -U $(EFUSE)
 
