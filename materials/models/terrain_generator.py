#!/usr/bin/python
# -*- coding: utf-8 -*-
from random import random

#Parameters:
size= 10.0
z_var= 0.05
cols= 100 # of x-points of grid
rows= 100 # of y-points of grid

vertex_count= rows*cols
index_count= (rows-1)*2*(cols-1) * 3

vertices= [0.0]*(vertex_count*3)
indices= [0.0]*index_count

cstep= size/(cols-1)
rstep= size/(rows-1)
for c in range(0,cols):
  for r in range(0,rows):
    vertices[3*(c*rows+r)+0]= cstep*c # + cstep*0.4*(random()-0.5)
    vertices[3*(c*rows+r)+1]= rstep*r # + rstep*0.4*(random()-0.5)
    vertices[3*(c*rows+r)+2]= z_var*random()

for c in range(0,cols-1):
  for r in range(0,rows-1):
    indices[3*(2*(c*(rows-1)+r)+0)+0]= c*rows+r
    indices[3*(2*(c*(rows-1)+r)+0)+1]= (c+1)*rows+(r+1)
    indices[3*(2*(c*(rows-1)+r)+0)+2]= c*rows+(r+1)

    indices[3*(2*(c*(rows-1)+r)+1)+0]= c*rows+r
    indices[3*(2*(c*(rows-1)+r)+1)+1]= (c+1)*rows+r
    indices[3*(2*(c*(rows-1)+r)+1)+2]= (c+1)*rows+(r+1)

print '// size= '+str(size)+'; z_var= '+str(z_var)+'; cols= '+str(cols)+'; rows= '+str(rows)
print 'Vertices=('
eol=','
for i in range(0,vertex_count):
  if(i==vertex_count-1):  eol=""
  print u"%f, %f, %f%s" % (vertices[3*i],vertices[3*i+1],vertices[3*i+2], eol)
print ')'

print 'Indices=('
eol=','
for i in range(0,index_count/3):
  if(i==index_count/3-1):  eol=""
  print u"%d, %d, %d%s" % (indices[3*i],indices[3*i+1],indices[3*i+2], eol)
print ')'
