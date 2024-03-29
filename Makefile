INCFLAGS  = -I /usr/include/GL
INCFLAGS += -I /usr/include/vecmath
# INCFLAGS += -I /mit/6.837/public/include/vecmath

LINKFLAGS = -lglut -lGL -lGLU
LINKFLAGS += -L /usr/lib -lvecmath
# LINKFLAGS += -L /mit/6.837/public/lib -lvecmath

CFLAGS    = -O2 -Wall -ansi -DSOLN -std=c++11
CC        = g++
SRCS      = main.cpp parse.cpp curve.cpp surf.cpp camera.cpp
OBJS      = $(SRCS:.cpp=.o)
PROG      = a1

all: $(SRCS) $(PROG)

$(PROG): $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o $@ $(LINKFLAGS)

.cpp.o:
	$(CC) $(CFLAGS) $< -c -o $@ $(INCFLAGS)

depend:
	makedepend $(INCFLAGS) -Y $(SRCS)

clean:
	rm $(OBJS) $(PROG)
