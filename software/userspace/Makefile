CC=arm-linux-gnueabihf-gcc
CFLAGS=-Wall -O0 -g

all: readstate interrupt

%: %.c
	$(CC) $(CFLAGS) $< $(LDFLAGS) -o $@
