DIALECT = -std=c11
CFLAGS += $(DIALECT) -O2 -g -W -Wall -D_GNU_SOURCE
LIBS = -lm -lpthread

CFLAGS += $(shell pkg-config --cflags libiio libad9361)

all: pluto-gps-sim
	
%.o: %.c *.h
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

pluto-gps-sim: plutogpssim.o $(COMPAT)
	${CC} $< ${LDFLAGS} $(LIBS) -o $@ $(shell pkg-config --cflags libiio libad9361) $(shell pkg-config --libs libiio libad9361)

clean:
	rm -f *.o  pluto-gps-sim
