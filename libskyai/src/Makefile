##-------------------------------------------------------------------------------------------
#     Copyright (C) 2010  Akihiko Yamaguchi
#
#     This file is part of SkyAI.
#
#     This program is free software: you can redistribute it and/or modify
#     it under the terms of the GNU General Public License as published by
#     the Free Software Foundation, either version 3 of the License, or
#     (at your option) any later version.
#
#     This program is distributed in the hope that it will be useful,
#     but WITHOUT ANY WARRANTY; without even the implied warranty of
#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#     GNU General Public License for more details.
#
#     You should have received a copy of the GNU General Public License
#     along with this program.  If not, see <http://www.gnu.org/licenses/>.
##-------------------------------------------------------------------------------------------
# EXEC = GENERATING_PROGRAM_NAME
ARC = ../lib/libskyai.a
LIBOBJS =
CXX = g++
AR = ar
CXXFLAGS = -g -Wall -rdynamic -O2 -march=i686
LDFLAGS  = $(CXXFLAGS)
RMFLAGS  =  -vf
##-------------------------------------------------------------------------------------------
BASEPREFIX := $(wildcard ~/prg)
LIBLORA := ../../liblora
LIBSKYAI := ..
##-------------------------------------------------------------------------------------------
## make  *.o  from  *.c and *.cpp
OBJS    = \
      $(patsubst %.c,%.o,$(wildcard *.c)) \
      $(patsubst %.cpp,%.o,$(wildcard *.cpp)) #
MOBJS    = \
      $(patsubst %.c,%.o,$(wildcard */*.c)) \
      $(patsubst %.cpp,%.o,$(wildcard */*.cpp)) #
##-------------------------------------------------------------------------------------------
LIBS    = -I/usr/include -I/usr/local/include -I$(LIBSKYAI)/include
LDLIBS  = -lm -L/usr/local/lib
##-------------------------------------------------------------------------------------------
## for liblora
LIBS    += -I$(LIBLORA)/include
LIBOBJS += $(LIBLORA)/lib/liblora.a
##-------------------------------------------------------------------------------------------
## for Octave
# LIBS   += -I/usr/include/octave-`octave-config -v`
# LDLIBS += -L/usr/lib/octave-`octave-config -v` -loctave -lcruft  -Wl,-rpath /usr/lib/octave-`octave-config -v`
# LDLIBS += -ldl -lfftw3 -L/usr/lib/atlas -latlas -llapack -lblas
##-------------------------------------------------------------------------------------------
## for ODE
# include $(LIBLORA)/src/Makefile_odeflags
# LDLIBS += -lSM -lICE -lGL -L/usr/X11R6/lib -lXext -lX11 -ldl -lGLU -lpthread
##-------------------------------------------------------------------------------------------
# all : libobjs $(EXEC)
all : libobjs $(ARC) $(MOBJS)
##-------------------------------------------------------------------------------------------
## additional make
libobjs :
	@(set -e; make -C $(LIBLORA)/src std oct)
# 	@(set -e; make -C $(LIBNGNET))
##-------------------------------------------------------------------------------------------
## generic make rules

clean   :
	@rm $(RMFLAGS) *.o *.a *.il $(OBJS) $(MOBJS) $(EXEC) $(ARC) ../lib/*.o $(EXTRA_CLEAN)
	@find -name '*.d' -type f | xargs rm $(RMFLAGS)

# $(EXEC) : $(OBJS) $(LIBOBJS)
# 	$(CXX) -o $@ $(OBJS) $(LDFLAGS) $(LIBOBJS) $(LDLIBS)
# 	@(echo "$(EXEC) is generated")

$(ARC) : $(OBJS)
	@(mkdir -p ../lib)
	$(AR) r $(ARC) $(OBJS)
	@(echo "$(ARC) is generated")

# $(ARC) : $(OBJS) $(LIBOBJS)
# 	$(AR) r $(ARC) $(OBJS) $(LIBOBJS)
# 	@(echo "$(ARC) is generated")

%.o : %.cpp
	$(CXX) -o $@ $(LIBS) $(CXXFLAGS) -c $<
	@(if [ -z "`echo '$@' | sed 's/[^/]*\\/[^/]*//g'`" ];then \
	cp -a $@ ../lib/; \
	echo "../lib/`basename $@` is generated"; \
	fi; \
	)

# 	(fn="`echo '$@'|sed 's/[/._]/\\\\\0/g'`"; \
# 	if [ -z "`echo '$(MOBJS)' | sed 's/.*\b\${fn}\b.*//g'`" ];then \
# 	echo "$@ is generated"; \
# 	echo "copy $@ to ../lib/`basename $@`"; \
# 	fi; \
# 	)

DEPOPT  ?= -MM

%.d: %.cpp
	@(rm -f $@; \
	$(CXX) $(DEPOPT) $(LIBS) $(CXXFLAGS) $< > $@.$$$$; \
	sed "s,\(`basename $*`\)\.o[ :]*,$*.o $@ : ,g" < $@.$$$$ > $@; \
	[ -s $@ ] || rm -f $@; \
	rm -f $@.$$$$; \
	)
##-------------------------------------------------------------------------------------------
include $(OBJ:.o=.d) $(OBJS:.o=.d) $(MOBJS:.o=.d) $(objects:.o=.d) $(addsuffix .d, $(BINARY))
##-------------------------------------------------------------------------------------------
