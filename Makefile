#
# Makefile for hex-heli quad copter
#
# Author: Tim Menninger
#

CC       = gcc
CFLAGS   = -g -Wall -Wstrict-prototypes -Wlong-long -Wcomment -ansi -pedantic
INCLUDE  = -Iinc/
TESTINC  = -Isrc/
SRCDIR   = src/
TESTDIR  = test/
BINDIR   = bin/

SOURCES  = $(SRCDIR)*.c
TESTSRC  = $(TESTDIR)*.c $(shell find $(SRCDIR) ! -name "main.*" -name "*.c")
EXENAME  = $(BINDIR)hexheli
TESTNAME = $(BINDIR)test_hexheli

all: hexheli test_no_run

hexheli: $(SOURCES)
	$(CC) $(CFLAGS) -o $(EXENAME) $(INCLUDE) $(SOURCES)
	make tidy

test_no_run: $(TESTSRC)
	$(CC) $(CFLAGS) -o $(TESTNAME) $(INCLUDE) $(TESTINC) $(TESTSRC)
	make tidy

test: test_no_run
	./$(TESTNAME)

tidy:
	rm -rf *.o $(BINDIR)*.dSYM *.DS_Store*

clean:
	rm -rf *.o $(EXENAME) $(TESTNAME) $(BINDIR)*.dSYM .DS_Store
