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
BASE_REL_DIR:=../..
include $(BASE_REL_DIR)/Makefile_preconf
##-------------------------------------------------------------------------------------------
EXEC := bioloid.out
##-------------------------------------------------------------------------------------------
## make  *.out  from  *.c and *.cpp
OBJS := \
      $(patsubst %.c,%.o,$(wildcard *.c)) \
      $(patsubst %.cpp,%.o,$(wildcard *.cpp)) #
##-------------------------------------------------------------------------------------------
USING_SKYAI_CV:=true
MAKING_SKYAI:=true
include $(BASE_REL_DIR)/Makefile_body
##-------------------------------------------------------------------------------------------
lib : libbioloid.o
##-------------------------------------------------------------------------------------------
