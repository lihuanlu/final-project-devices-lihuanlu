CROSS_COMPILE ?=
CC := $(CROSS_COMPILE)gcc
CFLAGS ?= -O0 -g -Wall -Werror
LDFLAGS ?=
LIBS ?= 

# Default target
all:	tmpcontrol

tmpcontrol: tmpcontrol.o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

%.o: %.c
	$(CC) $(CFLAGS) -c $<
	
clean:
	rm -f *.o
	rm -f tmpcontrol

