#!/usr/bin/python
# This program analyze a debug-dump file which is generated by general_agent.out with the "-dump_debu true" option. Usually, the name of the dump-debug file is "debug.dat"
usage='analyze_debug_dump.py DEBUG_DUMP'

import sys,os,re

if len(sys.argv)!=2:
  print 'usage: '+usage
  sys.exit(1)

replacing_index=0

MODE_NORMAL=0
MODE_PORT_ASSIGN=1

mode=MODE_NORMAL
port_db={}

re_port_id= re.compile(r'^port\s+(0x[0-9a-fA-F]+)')
re_port_name= re.compile(r'^\s+([_.0-9a-zA-Z]+)')
re_port_id_replace= re.compile(r'(0x[0-9a-fA-F]+)')

pfile=open(sys.argv[1])

def replace_port_id(port_id):
  if port_id in port_db:
    return port_db[port_id][replacing_index]
  return port_id

while(1):
  line=pfile.readline()
  if(not line):  break

  if mode==MODE_PORT_ASSIGN:
    obj= re_port_name.match(line)
    if obj:
      port_db[port_id]+= [obj.group(1)]
    else:
      mode= MODE_NORMAL

  if mode==MODE_NORMAL:
    obj= re_port_id.match(line)
    if obj:
      mode= MODE_PORT_ASSIGN
      port_id=obj.group(1)
      port_db[port_id]= []
    else:
      line= re_port_id_replace.sub((lambda obj: replace_port_id(obj.group(0))),line)

  print line,

  #line=re.sub(r'red','#ff0000',line)
  #line=re.sub(r'green','#008000',line)
  #line=re.sub(r'blue','#0000ff',line)
  #line=re.sub(r'lime','#00ff00',line)

  #line=re.sub(r'#(?P<R>[0-9a-fA-F]{2,2})(?P<G>[0-9a-fA-F]{2,2})(?P<B>[0-9a-fA-F]{2,2})', \
         #lambda obj:convert_colorcode_to_gray(obj.group('R'),obj.group('G'),obj.group('B')), line)
  #if len(sys.argv)!=3:  print line,
  #else:                 pofile.write(line+'\n')

#print port_db

pfile.close()
