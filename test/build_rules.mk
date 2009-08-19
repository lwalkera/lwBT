ifeq ($(ARMBUILD),1)
CC=arm-linux-gcc
else
CC=gcc
endif

.c.o: 
	@echo --[CC]-- $<
	@$(CC) $(CFLAGS) -c $< -o $@
%.out:
	@echo --[OUT]-- $^
	@$(CC) $(CFLAGS) $^ $(LDFLAGS)  -o $@
%.a:
	@echo --[AR]-- $@
	@ar rcs $@ $^
%.d: %.c
	@echo --[DEP]-- $<
	@$(SHELL) -ec '$(CC) -MM -MQ $*.o $(CFLAGS) $< > $@'


