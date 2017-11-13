CC=arm-linux-gnueabihf-gcc
CFLAGS=-Wall -O0 -g

all: readstate 

%: %.c
	$(CC) $(CFLAGS) $< $(LDFLAGS) -o $@
