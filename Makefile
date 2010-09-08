##-------------------------------------------------------------------------------------------
# \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
# \version 0.1
# \date    Jun. 08, 2010
##-------------------------------------------------------------------------------------------

all     : lora skyai bmks tls

clean	: TARGET_FLAG =clean
clean	: all

lora :
	@(set -e; make -C liblora/src $(TARGET_FLAG))

lora_std :
	@(set -e; make -C liblora/src std)

lora_oct :
	@(set -e; make -C liblora/src oct)

lora_ode :
	@(set -e; make -C liblora/src ode)

skyai :
	@(set -e; make -C libskyai/src $(TARGET_FLAG))

bmks :
	@(set -e; make -C benchmarks/maze2d $(TARGET_FLAG))
	@(set -e; make -C benchmarks/humanoid01 $(TARGET_FLAG))
	@(set -e; make -C benchmarks/bioloid $(TARGET_FLAG))

tls :
	@(set -e; make -C tools/ngnet-generator $(TARGET_FLAG))

##------------------------------------------------------------------
