IDIR = ../include
IDIR2 = /usr/lib/aarch64-linux-gnu
IDIR3 = /usr/include/c++/12
IGST = /usr/include/gstreamer-1.0
IGLIB = /usr/include/glib-2.0
IGLIB2 = $(IDIR2)/glib-2.0/include

CC=g++
CFLAGS=-I$(IDIR) -I$(DIR2) -I$(DIR3) -I$(IGST) -I$(IGLIB) -I$(IGLIB2)

ODIR=obj
LDIR =../lib;/usr/lib/aarch64-linux-gnu

LIBS=-lm -lgstreamer-1.0 -lgobject-2.0

#_DEPS = main.h
#DEPS = $(patsubst %,$(IDIR)/%,$(_DEPS))

_OBJ = main.o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))


$(ODIR)/%.o: %.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

DASPi_Trial_1: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~ 
