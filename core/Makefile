# ref: http://hiltmon.com/blog/2013/07/03/a-simple-c-plus-plus-project-structure/
# ref: http://stackoverflow.com/questions/1139271/makefiles-with-source-files-in-different-directories

CC := g++ # This is the main compiler
# CC := clang --analyze # and comment out the linker last line for sanity

BUILDDIR := build
TARGET := bin/libdqn.a

SOURCES := bloom.c protocol.cpp
OBJECTS := $(BUILDDIR)/bloom.o $(BUILDDIR)/protocol.o

CFLAGS := -DRASPBERRY_PI -g -lm -Wall
LIB := -lwiringPi
INC := -I. -I./RHutil -I../RadioHead

$(TARGET): $(OBJECTS)
	@mkdir -p bin
	@echo " Linking..."
	@echo " $(AR) rcs $(TARGET) $(OBJECTS)"; $(AR) rcs $(TARGET) $(OBJECTS);

$(BUILDDIR)/%.o: %.c
	@mkdir -p $(BUILDDIR)
	@mkdir -p $(@D)
	@echo " gcc $(CFLAGS) $(INC) -c -o $@ $<"; gcc $(CFLAGS) $(INC) -c -o $@ $<

$(BUILDDIR)/%.o: %.cpp
	@mkdir -p $(BUILDDIR)	
	@mkdir -p $(@D)
	@echo " g++ $(CFLAGS) $(INC) -c -o $@ $<"; g++ $(CFLAGS) $(INC) -c -o $@ $<

clean:
	@echo " Cleaning...";
	@echo " $(RM) -r $(BUILDDIR) $(TARGET)"; $(RM) -r $(BUILDDIR) $(TARGET)


.PHONY: clean
