#!/bin/bash
usage="
Interface for humanoid01.out to replace XXXRESDIR in agent file(s)
Options:
  `basename $0`
      -outdir OUTPUT_DIR()
      -agent AGENT_FILE1[,AGENT_FILE1[,..]]()
      [-help 1]"
#--------------------------------------------------

agent_files=
outdir=
exec_command=./humanoid01.out

#--------------------------------------------------
# bits
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
# parse option
#--------------------------------------------------

new_options=

while true; do
  case "$1" in
    -outdir)
      outdir="$2";
      new_options="$new_options $1 $2";
      shift 2 ;;
    -agent)
      agent_files="$2";
      shift 2 ;;
    -help)
      echo "usage: $usage";
      new_options="$new_options $1 $2";
      shift 2 ;;
    '')
      shift; break ;;
    *)
      new_options="$new_options $1 $2";
      shift 2 ;;
  esac
done

if [ -z "$agent_files" ];then
  echo "error! specify agent file(s) by -agent XXX option."
  echo ''
  echo "usage: $usage"
  exit 1
fi
if [ -z "$outdir" ];then
  echo "error! specify output directory by -outdir XXX option."
  echo ''
  echo "usage: $usage"
  exit 1
fi

if [ -e "$outdir" ] && [ -n "$(ls $outdir)" ];then
  echo "$outdir already exists. will you overwrite?"
  if ask-yes-no; then
    echo "continue."
  else
    echo "exit."
    exit 0
  fi
fi

#--------------------------------------------------
# modify agent files
#--------------------------------------------------

new_agent_files=
delim=''

escoutdir=$(echo $outdir | sed 's/\([/!@#$%^&*()_+=|\`~:;,.?]\)/\\\1/g')

tmp_agent_dir=/tmp/rl$$
mkdir $tmp_agent_dir
for f in ${agent_files//,/ };do
  f2=$(basename $f)
  sed "s/XXXRESDIR\//$escoutdir/g" $f > $tmp_agent_dir/$f2
  new_agent_files="$new_agent_files$delim$tmp_agent_dir/$f2"
  delim=','
done

#--------------------------------------------------
# execute
#--------------------------------------------------

if [ -e ${outdir}agent-in ];then
  rm -rv ${outdir}agent-in
fi

cp -av $tmp_agent_dir ${outdir}agent-in

echo "$exec_command -agent $new_agent_files $new_options"
$exec_command -agent $new_agent_files $new_options

rm -rv $tmp_agent_dir

