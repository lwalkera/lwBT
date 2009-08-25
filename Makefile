CFLAGS = -I. -Wall -m32 -g

all: lwbt.out

include build_rules.mk

sources = $(shell cat filelist)
ifneq ($(MAKECMDGOALS),mrproper)
ifneq ($(MAKECMDGOALS),clean)
-include $(sources:.c=.d)
endif
endif

lwbt.out: $(sources:.c=.o)

.PHONY: clean mrproper
clean:
	rm -f *.o *.out
	rm -f lwip/*.o lwip/*.out
	rm -f lwbt/*.o lwbt/*.out
mrproper: clean
	rm -f *.d
	rm -f lwip/*.d
	rm -f lwbt/*.d
