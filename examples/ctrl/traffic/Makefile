# After running make all, use shift + command + period on Mac to see hidden .libs folder 
# Paste .so into controllers folder to use it in worldfile
# Makefile explainer: https://opensource.com/article/18/8/what-how-makefile

.PHONY = all clean


CXX = gcc
CXXLD = gcc
LIBTOOL = glibtool
STAGE_CFLAGS = `pkg-config --cflags stage`

# included folders
INC = -I/opt/homebrew/Cellar/fltk/1.3.8_1/include/ -I/opt/homebrew/include/


SRCS := $(wildcard *robot*.cc)
BINS := $(SRCS:%.cc=%.la)

all: ${BINS}

%.la: %.lo traffic_robots.lo
	$(LIBTOOL) --tag CXX --mode=link $(CXXLD) $(INC) -std=c++17 -rpath /Users/lucyliu/stg/lib/Stage-4.3 -g -O -o $@ $^ -module -export-dynamic

%.lo: %.cc
	$(LIBTOOL) --tag CXX --mode=compile $(CXX) $(INC) -std=c++17 -g -O -c $<

clean:
	rm -rf *.o *.lo *.la .libs 

