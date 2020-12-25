CC=gcc
CFLAGS=-I /usr/include/libevdev-1.0
LDFLAGS=$(CFLAGS)
LDLIBS=-levdev -lpthread
OBJS=evdevd.o
TARGET=evdevd

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $(OBJS) $(LDLIBS)

evdevd.o: evdevd.c

