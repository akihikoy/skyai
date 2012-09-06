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
TARGET_FLAG :=

all     : l lcv s sb

debug	: TARGET_FLAG :=debug
debug	: all

clean	: TARGET_FLAG :=clean
clean	: all

l :
	@(set -e; make -f Makefile_l $(TARGET_FLAG))

lcv :
	@(set -e; make -f Makefile_lcv $(TARGET_FLAG))

s :
	@(set -e; make -f Makefile_s $(TARGET_FLAG))

sb :
	@(set -e; make -f Makefile_sb $(TARGET_FLAG))

##------------------------------------------------------------------
