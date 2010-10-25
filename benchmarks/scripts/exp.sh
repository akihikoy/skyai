#!/bin/bash
usage="./exp.sh OPTIONS COND_ID1 AGENT_FILE1 COND_ID2 AGENT_FILE2 COND_ID3 AGENT_FILE3 ...
Execute experiments from START_INDEX to END_INDEX for each AGENT_FILE
OPTIONS:
  `basename $0`
      -o RESULT_DIR()
      -x|-exec EXEC_COMMAND()
      [-opt  options for EXEC_COMMAND()]
      [-s START_INDEX(0)]
      [-e END_INDEX(9)]
      [-help]"
#--------------------------------------------------

cond_num=0
result_dir=
exec_command=
exec_options=
startindex=0
endindex=9

cmdline="$0 $@"

#--------------------------------------------------
# parse option
#--------------------------------------------------

while true; do
  case "$1" in
    -o) result_dir="$2"; shift 2 ;;
    -s) startindex="$2"; shift 2 ;;
    -e) endindex="$2"; shift 2 ;;
    -x|-exec) exec_command="$2"; shift 2 ;;
    -opt) exec_options="$exec_options $2"; shift 2 ;;
    -help|--help) echo "usage: $usage"; exit 0 ;;
    '') shift; break ;;
    *)
      cond_id[$cond_num]="$1"
      agent_file[$cond_num]="$2"
      cond_num=$(($cond_num+1))
      shift 2 ;;
  esac
done

if [ $cond_num -eq 0 ];then
  echo "error! specify agent file(s)."
  echo ''
  echo "usage: $usage"
  exit 1
fi
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

#--------------------------------------------------
# ask user

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
# confirm

echo '--------------------------------------------------'
echo "cond_num       = $cond_num    "
echo "cond_id        = ${cond_id[@]}"
echo "agent_file     = ${agent_file[@]}"
echo "result_dir     = $result_dir  "
echo "exec_command   = $exec_command"
echo "exec_options   = $exec_options"
echo "startindex     = $startindex  "
echo "endindex       = $endindex    "
echo '--------------------------------------------------'

echo "start with this setup?"
if ask-yes-no; then
  echo "start..."
else
  echo "exit."
  exit 0
fi

#--------------------------------------------------
# exec loop

function rlexp
{
  # in: resdir, cond_idx, trial_idx
  mkdir $resdir
  exec_cmdline="$exec_command $exec_options -agent ${agent_file[$cond_idx]} -outdir $resdir"
  echo '--------------------------------------------------'
  echo "start experiment ${cond_id[$cond_idx]} #$trial_idx"
  echo "  in $resdir"
  echo "  with cmdline: $exec_cmdline"
  if $exec_cmdline; then
    echo "done"
    echo '--------------------------------------------------'
    echo ''
  else
    echo '**************************************************'
    echo "$exec_command terminated with an error code!"
    echo '**************************************************'
    exit 1
  fi
}

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

for ((i=0;i<$cond_num;i++)); do
  if [ ! -z "${cond_id[$i]}" ]; then
    rdir[$i]=$result_dir/${cond_id[$i]}
    mkdir -p ${rdir[$i]}
  fi
done

#--------------------------------------------------
# copy the execfile, cmdline, cond dir

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
echo "$cmdline" > /tmp/cmdline
copy_file /tmp/cmdline
rm /tmp/cmdline

#--------------------------------------------------
# running experiment loop

for (( n=$startindex; n<=$endindex; n++ )); do
  for ((i=0;i<$cond_num;i++)); do
    if [ ! -z "${cond_id[$i]}" ]; then
      resdir=${rdir[$i]}/ex$n
      if [ -d $resdir ]; then
        echo "$resdir is exists. skipped..."
      else
        cond_idx=$i
        trial_idx=$n
        rlexp
      fi
    fi
  done
done

#--------------------------------------------------
