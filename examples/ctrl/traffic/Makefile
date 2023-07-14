# After running make all, use shift + command + period on Mac to see hidden .libs folder 
# Pass .so into controllers folder to use it in worldfile

.PHONY = all clean

CXX = gcc
CXXLD = gcc
LIBTOOL = glibtool
STAGE_CFLAGS = `pkg-config --cflags stage`

SRCS := $(wildcard *.cc)
BINS := $(SRCS:%.cc=%.la)

all: ${BINS}

%.la: %.lo
	$(LIBTOOL) --tag CXX --mode=link $(CXXLD) -std=c++11 -rpath /Users/lucyliu/stg/lib/Stage-4.3 -module -export-dynamic -version-info 0:0:0 $^ -o $@ -dynamiclib

%.lo: %.cc
	$(LIBTOOL) --tag CXX --mode=compile $(CXX) -std=c++11 -g -O -c $<

clean:
	rm -rf *.o *.lo *.la
#	rm -rf *.o *.lo *.la .libs  

