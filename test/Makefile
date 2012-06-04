##-------------------------------------------------------------------------------------------
#     Copyright (C) 2012  Akihiko Yamaguchi
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
BASE_REL_DIR:=..
include $(BASE_REL_DIR)/Makefile_preconf
##-------------------------------------------------------------------------------------------
## make  *.out  from  *.c and *.cpp
USER_MAKE_TRG := execs
EXECS    := \
      $(patsubst %.c,%.out,$(wildcard *.c)) \
      $(patsubst %.cpp,%.out,$(wildcard *.cpp)) #
OBJS := $(EXECS:.out=.o)
##-------------------------------------------------------------------------------------------
USING_SKYAI_ODE:=true
MAKING_SKYAI:=true
include $(BASE_REL_DIR)/Makefile_body
##-------------------------------------------------------------------------------------------
execs : $(EXECS)

%.out : %.o $(LIBOBJS)
	$(CXX) -o $@ $< $(LDFLAGS) $(LIBOBJS) $(LDLIBS)
	@(echo "*************************************************")
	@(echo "$@ is generated")
	@(echo "*************************************************")
##-------------------------------------------------------------------------------------------
