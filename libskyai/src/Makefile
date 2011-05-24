##-------------------------------------------------------------------------------------------
#     Copyright (C) 2010,2011  Akihiko Yamaguchi
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
include ../../Makefile_preconf
##-------------------------------------------------------------------------------------------
ARC := ../lib/libskyai.a
##-------------------------------------------------------------------------------------------
OBJS    := \
      $(patsubst %.c,%.o,$(wildcard *.c)) \
      $(patsubst %.cpp,%.o,$(wildcard *.cpp)) #
MOBJS    := \
      $(patsubst %.c,%.o,$(wildcard */*.c)) \
      $(patsubst %.cpp,%.o,$(wildcard */*.cpp)) #
##-------------------------------------------------------------------------------------------
EXTRA_CLEAN+= $(MOBJS) ../lib/*.o
##-------------------------------------------------------------------------------------------
USING_LORA_OCT:=true
USING_SKYAI:=true
include ../../Makefile_postconf
##-------------------------------------------------------------------------------------------
# modify the rules:
all : $(MOBJS)

%.o : %.cpp
	$(CXX) -o $@ $(LIBS) $(CXXFLAGS) -c $<
	@(if [ -z "`echo '$@' | sed 's/[^/]*\\/[^/]*//g'`" ];then \
	ln -sfv ../src/$@ ../lib/; \
	echo "*************************************************";\
	echo "../lib/`basename $@` is generated"; \
	echo "*************************************************";\
	fi; \
	)
##-------------------------------------------------------------------------------------------
include $(MOBJS:.o=.d)
##-------------------------------------------------------------------------------------------
