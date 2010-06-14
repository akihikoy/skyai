#!/bin/bash
usage="
Load conditions directory and execute an experiment.
Conditions directory:
  - put agent files with the extension .agent
  - common.agent is commonly used in every condition
  - common-XXX.agent is commonly used in runs of XXX group
  - an agent file XXXYYY.agent denotes a condition XXXYYY which belongs to XXX group
  - each condition XXXYYY is executed (END_INDEX-START_INDEX+1) times
  - experimental data of a condition XXXYYY is stored to a directory RESULT_DIR/XXXYYY/
Options:
  `basename $0`
      -o RESULT_DIR()
      -x|-exec EXEC_COMMAND()
      -c COND_DIR()
      -opt  options for EXEC_COMMAND()
      [-s START_INDEX(0)]
      [-e END_INDEX(9)]
      [-help]"
#--------------------------------------------------

result_dir=
exec_command=
conditions_dir=
exec_options=
startindex=0
endindex=9

commandline="$0 $@"

#--------------------------------------------------
# parse option
#--------------------------------------------------

while true; do
  case "$1" in
    -o) result_dir="$2"; shift 2 ;;
    -s) startindex="$2"; shift 2 ;;
    -e) endindex="$2"; shift 2 ;;
    -x|-exec) exec_command="$2"; shift 2 ;;
    -c) conditions_dir="$2"; shift 2 ;;
    -opt) exec_options="$2"; shift 2 ;;
    -help|--help) echo "usage: $usage"; exit 0 ;;
    '') shift; break ;;
    *) echo "unexpected option '$1'; please report a bug." >&2
        exit 1 ;;
  esac
done

if [ -z "$result_dir" ];then
  echo "error! specify the result directory by -o XXX option."
  echo ''
  echo "usage: $usage"
  exit 1
fi
if [ -z "$exec_command" ];then
  echo "error! specify the exec command by -x XXX option."
  echo ''
  echo "usage: $usage"
  exit 1
fi
if [ -z "$conditions_dir" ];then
  echo "error! specify the conditions directory by -c XXX option."
  echo ''
  echo "usage: $usage"
  exit 1
fi

echo "result_dir     = $result_dir  "
echo "exec_command   = $exec_command"
echo "exec_options   = $exec_options"
echo "conditions_dir = $conditions_dir"
echo "startindex     = $startindex  "
echo "endindex       = $endindex    "

#--------------------------------------------------
# check conditions directory
#--------------------------------------------------

COND_NUM=1
f0=$conditions_dir/common.agent
if ! [ -f $f0 ];then
  f0=
fi
for f1 in $conditions_dir/common-*.agent; do
  f2=$(basename $f1 .agent)
  group=${f2/common-/}
  for f3 in $conditions_dir/$group*.agent; do
    suffix[$COND_NUM]=$(basename $f3 .agent)
    agent_files[$COND_NUM]="$f0 $f1 $f3"
    COND_NUM=$(($COND_NUM+1))
  done
done

#--------------------------------------------------
# exec loop
#--------------------------------------------------

function rlexp
{
  # in: resdir, ex_id, trials
  mkdir $resdir
  mkdir $resdir/conds
  mkdir $resdir/rl
  e_agent_files=
  for cf in ${agent_files[$ex_id]}; do
    bcf=$(basename $cf)
    rd=$(echo $resdir/rl | sed 's/\([/!@#$%^&*()_+=|\`~:;,.?]\)/\\\1/g')
    sed "s/XXXRESDIR/$rd/g" $cf > $resdir/conds/$bcf
    e_agent_files="$e_agent_files $resdir/conds/$bcf"
  done
  # $exec_command $option_com ${option_i[$ex_id]} -outdir $resdir/rl/  # for rl-*
  echo "start experiment ${suffix[$ex_id]} #$trials"
  echo "  in $resdir"
  if $exec_command $exec_options -agent "$e_agent_files" -outdir $resdir/rl/; then
    echo "done"
    echo ''
    echo ''
  else
    echo "$exec_command terminated with an error code!"
    exit 1
  fi
}

#--------------------------------------------------
# ask user
#--------------------------------------------------

#!/bin/bash
function ask-yes-no()
{
  while true; do
    echo -n '  (y|n) > '
    read s
    if [ "$s" == "y" ];then return 0; fi
    if [ "$s" == "n" ];then return 1; fi
  done
}

#--------------------------------------------------
# source from $HOME/lib/bash/experiment-common.sh
#--------------------------------------------------

# +requirements:
# suffix[]: conditions directory name[]
# COND_NUM: num of conditions
# result_dir: base directory of results
# startindex: start index of an experment with a condition
# endindex: end index of an experment with a condition
# rlexp: function to perform an experiment with a condition

#--------------------------------------------------
# making directories

if [ -d $result_dir ]; then
  echo "directory $result_dir already exists."
  echo "continue?"
  if ask-yes-no; then
    echo "continue."
  else
    echo "exit."
    exit 0
  fi
fi

for ((i=1;i<=$COND_NUM;i++)); do
  if [ ! -z "${suffix[$i]}" ]; then
    rdir[$i]=$result_dir/${suffix[$i]}
    mkdir -p ${rdir[$i]}
  fi
done

# mkdir -p result/rl

#--------------------------------------------------
# copy the execfile, commandline, cond dir

function copy_file() # filename
{
  filename=$1
  filenamebn=`basename $filename`
  if [ -e $result_dir/$filenamebn ] && [ -n "`diff -x '*~' $filename $result_dir/$filenamebn`" ]; then
    for ((j=1; j<1000; j++)); do
      oldfile=$result_dir/old$j-$filenamebn
      if [ ! -e $oldfile ]; then
        mv $result_dir/$filenamebn $oldfile
        break
      fi
    done
  fi
  cp -aL $filename $result_dir
}

copy_file $0
echo "$commandline" > /tmp/exec
copy_file /tmp/exec
rm /tmp/exec
copy_file $conditions_dir

#--------------------------------------------------
# running experiment loop

for (( n=$startindex; n<=$endindex; n++ )); do
  for ((i=1;i<=$COND_NUM;i++)); do
    if [ ! -z "${suffix[$i]}" ]; then
      resdir=${rdir[$i]}/ex$n
      if [ -d $resdir ]; then
        echo "$resdir is exists. skipped..."
      else
        ex_id=$i
        trials=$n
        rlexp
      fi
    fi
  done
done

#--------------------------------------------------
