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

all     : lora skyai bmks tls

clean	: TARGET_FLAG :=clean
clean	: all

lora :
	@(set -e; make -C liblora/src $(TARGET_FLAG))

lora_std :
	@(set -e; make -C liblora/src std)

lora_oct :
	@(set -e; make -C liblora/src oct)

lora_ode :
	@(set -e; make -C liblora/src ode)

lora_cv :
	@(set -e; make -C liblora/src cv)

skyai :
	@(set -e; make -C libskyai/src $(TARGET_FLAG))

bmks :
	@(set -e; make -C benchmarks/maze2d $(TARGET_FLAG))
	@(set -e; make -C benchmarks/humanoid01 $(TARGET_FLAG))
	@(set -e; make -C benchmarks/bioloid $(TARGET_FLAG))

tls :
	@(set -e; make -C tools/ngnet-generator $(TARGET_FLAG))
	@(set -e; make -C tools/agent_talker $(TARGET_FLAG))

##------------------------------------------------------------------
