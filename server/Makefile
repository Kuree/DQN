# ref http://hiltmon.com/blog/2013/07/03/a-simple-c-plus-plus-project-structure/
# ref http://stackoverflow.com/questions/1139271/makefiles-with-source-files-in-different-directories

# This is the main compiler
CC := g++
# CC := clang --analyze # and comment out the linker last line for sanity

SRCDIR := src
BUILDDIR := build
DQNSERVER := bin/DQNserver
SRCEXT := cpp
SOURCES := $(shell find $(SRCDIR) -type f -name *.$(SRCEXT))
OBJECTS := $(patsubst $(SRCDIR)/%,$(BUILDDIR)/%,$(SOURCES:.$(SRCEXT)=.o))

#removed -DBCM2835_NO_DELAY_COMPATIBILITY, not needed?
# we actually want this becuase it allows wiringPi and BCM2835 to coexist

LIB := -lwiringPi -ldqn -lradiohead -lm
RADIOHEADBASE := ../RadioHead
DQNBASE := ../core
INC := -I$(RADIOHEADBASE) -I$(DQNBASE)
override CFLAGS += -DRASPBERRY_PI -g -Wall

$(DQNSERVER): $(BUILDDIR)/DQNserver.o ../core/bin/libdqn.a $(RADIOHEADBASE)/bin/libradiohead.a
	@echo "Linking..."
	$(CC) $(BUILDDIR)/DQNserver.o -o $(DQNSERVER) -L$(RADIOHEADBASE)/bin -L$(DQNBASE)/bin -Wl,-rpath=$(RADIOHEADBASE)/bin,-rpath=$(DQNBASE)/bin $(LIB)

$(BUILDDIR)/%.o: $(SRCDIR)/%.$(SRCEXT)
	@mkdir -p $(BUILDDIR)
	$(CC) $(CFLAGS) $(INC) -c -o $@ $<

clean:
	@echo " Cleaning...";
	$(RM) -r $(BUILDDIR) $(DQNSERVER)

# blink
blink:
	$(CC) $(CFLAGS) test/tester.cpp $(INC) $(LIB) -o bin/tester

.PHONY: clean $(RADIOHEADBASE)
