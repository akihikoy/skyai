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
# ARC = GENERATING_ARCHIVE
LIBOBJS =
CXX = g++
AR = ar
CXXFLAGS := -g -Wall -rdynamic -O2 -march=i686
LDFLAGS  := $(CXXFLAGS)
RMFLAGS  :=  -vf
##-------------------------------------------------------------------------------------------
BASEPREFIX := $(wildcard ~/prg)
##-------------------------------------------------------------------------------------------
## make  *.out  from  *.c and *.cpp
OBJS := \
      $(patsubst %.c,%.o,$(wildcard *.c)) \
      $(patsubst %.cpp,%.o,$(wildcard *.cpp)) #
EXECS := $(OBJS:.o=.out)
# EXECS    = \
#       $(patsubst %.c,%.out,$(wildcard *.c)) \
#       $(patsubst %.cpp,%.out,$(wildcard *.cpp)) #
##-------------------------------------------------------------------------------------------
LIBS    = -I/usr/include -I/usr/local/include
LDLIBS  = -lm -L/usr/local/lib
##-------------------------------------------------------------------------------------------
## for libskyai
# LIBSKYAI := ../../libskyai
# LIBS    += -I$(LIBSKYAI)/include
# LIBOBJS += $(LIBSKYAI)/lib/libskyai.a
# LIBOBJS+= $(LIBSKYAI)/lib/*.o $(LIBSKYAI)/lib/libskyai.a
##-------------------------------------------------------------------------------------------
## for oldngnet
# LIBNGNET := $(BASEPREFIX)/robotics/ngnet-smem
# LIBS   += -I$(LIBNGNET)
# LIBOBJS+= $(LIBNGNET)/ngnet_smem.o
##-------------------------------------------------------------------------------------------
## for liblora
LIBLORA := ../../liblora
LIBS    += -I$(LIBLORA)/include
LIBOBJS += $(LIBLORA)/lib/liblora_oct.a $(LIBLORA)/lib/liblora.a
##-------------------------------------------------------------------------------------------
## for Octave
LIBS   += -I/usr/include/octave-`octave-config -v`
LDLIBS += -L/usr/lib/octave-`octave-config -v` -loctave -lcruft -Wl,-rpath /usr/lib/octave-`octave-config -v`
LDLIBS += -ldl -lfftw3 -L/usr/lib/atlas -latlas -llapack -lblas
##-------------------------------------------------------------------------------------------
## for ODE
# LIBS   += -I$(BASEPREFIX)/libode/ode-0.10.1/include
# LIBS   += -DODE_MINOR_VERSION=10 -DdDOUBLE#for ODE-0.10.1
# LDLIBS += $(BASEPREFIX)/libode/ode-0.10.1/ode/src/.libs/libode.a
# LDLIBS += $(BASEPREFIX)/libode/ode-0.10.1/drawstuff/src/.libs/libdrawstuff.a
# LDLIBS += -lSM -lICE -lGL -L/usr/X11R6/lib -lXext -lX11 -ldl -lGLU -lpthread
##-------------------------------------------------------------------------------------------
# manoi01.o : LIBS += -I$(BASEPREFIX)/robotics/def-manoi01
# manoi01.d : LIBS += -I$(BASEPREFIX)/robotics/def-manoi01
# manoi01.o : CXXFLAGS += -DHUMANOID_VERSION=1
# manoi01.d : CXXFLAGS += -DHUMANOID_VERSION=1
##-------------------------------------------------------------------------------------------
all : lib $(EXECS)
# all : libobjs $(ARC)
##-------------------------------------------------------------------------------------------
## additional make
lib :
	@(set -e; make -C $(LIBLORA)/src std oct)
##-------------------------------------------------------------------------------------------
## generic make rules

clean   :
	@rm $(RMFLAGS) *.o *.a *.il $(EXECS) $(ARC) $(EXTRA_CLEAN)
	@find -name '*.d' -type f | xargs rm $(RMFLAGS)

# %.out : %.cpp $(LIBOBJS)
# 	$(CXX) -o $@ $< $(LIBS) $(CXXFLAGS) $(OBJS) $(LDFLAGS) $(LIBOBJS) $(LDLIBS)
# 	@(echo "$@ is generated")

%.out : %.o $(LIBOBJS)
	$(CXX) -o $@ $< $(LDFLAGS) $(LIBOBJS) $(LDLIBS)
	@(echo "$@ is generated")

# $(EXEC) : $(OBJS) $(LIBOBJS)
# 	$(CXX) -o $@ $(OBJS) $(LDFLAGS) $(LIBOBJS) $(LDLIBS)
# 	@(echo "$(EXEC) is generated")

# $(ARC) : $(OBJS) $(LIBOBJS)
# 	$(AR) r $(ARC) $(OBJS) $(LIBOBJS)
# 	@(echo "$(ARC) is generated")

%.o : %.cpp
	$(CXX) $(LIBS) $(CXXFLAGS) -o $@ -c $<

DEPOPT  ?= -M
# DEPOPT  ?= -MM

%.d: %.cpp
	@(rm -f $@; \
	$(CXX) $(DEPOPT) $(LIBS) $(CXXFLAGS) $< > $@.$$$$; \
	sed 's,\($*\)\.o[ :]*,\1.o $@ : ,g' < $@.$$$$ > $@; \
	[ -s $@ ] || rm -f $@; \
	rm -f $@.$$$$; \
	)
##-------------------------------------------------------------------------------------------
include $(OBJ:.o=.d) $(OBJS:.o=.d) $(objects:.o=.d) $(addsuffix .d, $(BINARY))
##-------------------------------------------------------------------------------------------
