# this is the top-level makefile!


all: RadioHead core server
	@echo "All done!"
	@ln -s server/bin/DQNserver

RadioHead:
	$(MAKE) -C RadioHead

core: RadioHead
	$(MAKE) -C core

server: RadioHead core
	$(MAKE) -C server

clean:
	$(MAKE) -C RadioHead clean
	$(MAKE) -C core clean
	$(MAKE) -C server clean

	rm DQNserver

.PHONY: RadioHead core server
