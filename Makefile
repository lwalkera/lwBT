CFLAGS = -I. -Wall -m32

all: lwbt.out

include build_rules.mk

sources = $(shell cat filelist)
ifneq ($(MAKECMDGOALS),mrproper)
ifneq ($(MAKECMDGOALS),clean)
-include $(sources:.c=.d)
endif
endif

lwbt.out: $(sources:.c=.o)

.PHONY: clean
clean:
	rm -f *.o *.d *.out
	rm -f lwip/*.o lwip/*.d lwip/*.out
	rm -f lwbt/*.o lwbt/*.d lwbt/*.out
