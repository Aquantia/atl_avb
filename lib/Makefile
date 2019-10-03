OBJS=atl hw_atl
INCL=atl_private.h atl.h 
AVBLIB=libatl.a
CFLAGS=-ggdb -DDEBUG

CC?=gcc
RANLIB?=ranlib

all: $(AVBLIB)

$(AVBLIB): $(addsuffix .o,$(OBJS))
	$(RM) $@
	$(AR) rcs $@ $^
	$(RANLIB) $@

atl.o: atl.c $(INCL)
	$(CC) -c $(INCFLAGS) $(CFLAGS) atl.c

hw_atl.o: hw_atl.c $(INCL)
	$(CC) -c $(INCFLAGS) $(CFLAGS) hw_atl.c


clean:
	$(RM) `find . -name "*~" -o -name "*.[oa]" -o -name "\#*\#" -o -name TAGS -o -name core -o -name "*.orig"`

