#!/bin/bash
usage="usage: compile.sh [options] FILES [-- OPTIONS GIVEN TO g++]
  options:
    -ww          : using extra warning options
    -lora        : using liblora
    -skyai       : using libskyai
    -oct         : using liboctave
    -ode         : using ODE
    -gl          : using OpenGL
    -make        : make libraries
    -x           : exe name
    -opts 'OPTS' : OPTS is directly given to g++
    -help        : show this"
##-------------------------------------------------------------------------------------------
CXX=g++
CXXFLAGS="-g -Wall -rdynamic -O2 -march=i686"
LIBS="-I/usr/include -I/usr/local/include"
LDLIBS="-lm -L/usr/local/lib"
skyai_dir=$(cd $(dirname $0)/../ && pwd)
ode_dir=$HOME/libode/ode-0.10.1
##-------------------------------------------------------------------------------------------
exe_name=
files=""
options=""
using_lora=0
using_skyai=0
using_oct=0
using_ode=0
using_gl=0
making_libs=0
##-------------------------------------------------------------------------------------------
while true; do
  case "$1" in
    -help|--help) echo "usage: $usage"; exit 0 ;;
    -ww)
        echo 'more warning options' > /dev/stderr
        CXXFLAGS+=" -Wshadow -Wpointer-arith -Wcast-qual -Wcast-align -Wwrite-strings -Wconversion -Woverloaded-virtual -Winline"
        shift ;;
    -lora  )  using_lora=1;  shift ;;
    -skyai )  using_skyai=1;  shift ;;
    -oct   )  using_oct=1;  shift ;;
    -ode   )  using_ode=1;  shift ;;
    -gl    )  using_gl=1;  shift ;;
    -make) making_libs=1;  shift ;;
    -x) exe_name=$1; shift 2 ;;
    -opts) options="$options $1"; shift 2 ;;
    ''|--) shift; break ;;
    *)
        if [ -z "$exe_name" ] && [ -z $files ];then
          exe_name=${1/.*/.out};
          if [ "$1" == "$exe_name" ];then exe_name=a.out; fi
        fi
        files="$files $1"
        shift 1
        ;;
  esac
done
options="$options $@"
##-------------------------------------------------------------------------------------------
## for libskyai
if [ $using_skyai -eq 1 ]; then
  echo 'using libskyai' > /dev/stderr
  LIBSKYAI="$skyai_dir/libskyai"
  LIBS+=" -I$LIBSKYAI/include"
  # LDLIBS+=" -L$LIBSKYAI/lib $LIBSKYAI/lib/*.o -lskyai"
  LDLIBS+=" -L$LIBSKYAI/lib -lskyai_mcore -lskyai_mstd -lskyai"
  LDLIBS+=" -Wl,-rpath $LIBSKYAI/lib"
  LDLIBS+=" -lboost_filesystem -lboost_regex"
fi
##-------------------------------------------------------------------------------------------
## for liblora in the skyai
if [ $using_lora -eq 1 ]; then
  echo 'using liblora' > /dev/stderr
  LIBLORA="$skyai_dir/liblora"
  LIBS+=" -I$LIBLORA/include"
  LDLIBS+=" -L$LIBLORA/lib"
  if [ $using_oct -eq 1 ]; then
    LDLIBS+=" -llora_oct"
  fi
  if [ $using_ode -eq 1 ]; then
    LDLIBS+=" -llora_ode"
  fi
  LDLIBS+=" -llora"
  LDLIBS+=" -Wl,-rpath $LIBLORA/lib"
fi
##-------------------------------------------------------------------------------------------
## for Octave
if [ $using_oct -eq 1 ]; then
  echo 'using liboctave' > /dev/stderr
  LIBS+=" -I/usr/include/octave-`octave-config -v`"
  LDLIBS+=" -L/usr/lib/octave-`octave-config -v` -loctave -lcruft -Wl,-rpath /usr/lib/octave-`octave-config -v`"
  #LDLIBS+=" -ldl -lfftw3 -L/usr/lib/atlas -latlas -llapack -lblas -lg2c"
  LDLIBS+=" -ldl -lfftw3 -L/usr/lib/atlas -latlas -llapack -lblas"
fi
##-------------------------------------------------------------------------------------------
## for ODE
if [ $using_ode -eq 1 ]; then
  echo 'using ODE' > /dev/stderr
  LIBS+=" -I$ode_dir/include"
  LIBS+=" -DODE_MINOR_VERSION=10 -DdDOUBLE" # for ODE-0.10.1
  LDLIBS+=" $ode_dir/ode/src/.libs/libode.a"
  LDLIBS+=" $ode_dir/drawstuff/src/.libs/libdrawstuff.a"
fi
##-------------------------------------------------------------------------------------------
## for OpenGL
if [ $using_gl -eq 1 ] || [ $using_ode -eq 1 ]; then
  echo 'using OpenGL' > /dev/stderr
  LDLIBS+=" -lSM -lICE -lGL -L/usr/X11R6/lib -lXext -lX11 -ldl -lGLU -lpthread"
fi
##-------------------------------------------------------------------------------------------

if [ $making_libs -eq 1 ]; then
  echo 'make libs...' > /dev/stderr
  if [ -n "${LIBSKYAI:-}" ];then
    echo "make -C $LIBSKYAI/src" > /dev/stderr
    if ! make -C $LIBSKYAI/src; then exit 1; fi
  fi
  if [ -n "${LIBLORA:-}" ];then
    echo "make -C $LIBLORA/src" > /dev/stderr
    if ! make -C $LIBLORA/src; then exit 1; fi
  fi
fi

echo "g++ -o $exe_name $CXXFLAGS $files $options $LIBS $LDLIBS" > /dev/stderr
g++ -o $exe_name $CXXFLAGS $files $options $LIBS $LDLIBS

