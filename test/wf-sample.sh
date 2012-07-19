#!/bin/bash
# before launching this script, execute:
#   make -f Makefile_s avf_plotter.out
#   ./avf_plotter.out -agent wf-sample > wf-sample.dat
# to execute this script, qplot is needed; but you can easily do the same thing with gnuplot
# qplot: http://d.hatena.ne.jp/aki-yam/20101009/1286612136

figfn=
while true; do
  case "$1" in
    -f) figfn="$2"; shift 2 ;;
    '') break ;;
    *) echo 'fatal arguments!'; exit 1
  esac
done

setting="set size 0.7,1"
setting+="; set xlabel 'State'"
setting+="; set ylabel 'Action'"
setting+="; set zlabel 'Value'"
setting+="; set parametric"
setting+="; set xrange [-1:1]"
setting+="; set yrange [-1.1:1]"
setting+="; set urange [-1:1]"
setting+="; set view 45,300,1,1.2"
setting+="; set pm3d"
# setting+="; set hidden3d"

# Gaussians:
h=0.3
sigma=7.11111111111111072
bf1="(exp(-0.5*(u-(-1))**2*$sigma))"
bf2="(exp(-0.5*(u-(0))**2*$sigma))"
bf3="(exp(-0.5*(u-(+1))**2*$sigma))"

# normalizer:
gsum="($bf1+$bf2+$bf3)"
# control wires:
aw1="u,-0.5,(0.0*$bf1/$gsum + 0.6*$bf2/$gsum + 0.0*$bf3/$gsum)"
aw2="u,0.5,(0.0*$bf1/$gsum + 0.3*$bf2/$gsum + 0.6*$bf3/$gsum)"

plot="wf-sample.dat w l t ''"
# basis functions (normalized Gaussian):
plot+=", u,-1.1,$h*$bf1/$gsum w l lt 3 t ''"
plot+=", u,-1.1,$h*$bf2/$gsum w l lt 3 t ''"
plot+=", u,-1.1,$h*$bf3/$gsum w l lt 3 t ''"
# control wires:
plot+=", $aw1 w l lt 2 lw 3 t ''"
plot+=", $aw2 w l lt 5 lw 3 t ''"

qplot -3d -s "$setting" -nc $plot

if [ -n "$figfn" ]; then
  echo "saving the graph into $figfn.."
  qplot -3d -s "$setting" -nc $plot -o $figfn
fi
