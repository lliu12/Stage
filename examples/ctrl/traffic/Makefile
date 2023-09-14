# After running make all, use shift + command + period on Mac to see hidden .libs folder 
# Paste .so into controllers folder to use it in worldfile
# Makefile explainer: https://opensource.com/article/18/8/what-how-makefile

.PHONY = all clean

CXX = gcc
CXXLD = gcc
LIBTOOL = glibtool
STAGE_CFLAGS = `pkg-config --cflags stage`

SRCS := $(wildcard *robot.cc)
BINS := $(SRCS:%.cc=%.la)

all: ${BINS}

%.la: %.lo base_robot.lo
#	$(LIBTOOL) --tag CXX --mode=link $(CXXLD) -std=c++17 -rpath /Users/lucyliu/stg/lib/Stage-4.3 -module -export-dynamic $^ -o $@ -dynamiclib
	$(LIBTOOL) --tag CXX --mode=link $(CXXLD) -std=c++17 -rpath /Users/lucyliu/stg/lib/Stage-4.3 -g -O -o $@ $^ -module -export-dynamic

%.lo: %.cc
	$(LIBTOOL) --tag CXX --mode=compile $(CXX) -std=c++17 -g -O -c $<

clean:
	rm -rf *.o *.lo *.la
#	rm -rf *.o *.lo *.la .libs  

