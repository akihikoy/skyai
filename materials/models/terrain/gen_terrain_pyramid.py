#!/usr/bin/python
# -*- coding: utf-8 -*-
from random import random

#Parameters:
x_size= 10.0
y_size= 10.0
pyrm_x= 0.15
pyrm_y= 0.15
pyrm_z= 0.02
cols= 50 # num of x-lines of the grid
rows= 50 # num of y-lines of the grid

vertices= []
indices= []

def add_pyramid(x,y,sizex,sizey,sizez):
  global vertices, indices
  idx= len(vertices)/3
  vertices+= [x+0.5*sizex]
  vertices+= [y+0.5*sizey]
  vertices+= [0.0]
  vertices+= [x+0.5*sizex]
  vertices+= [y-0.5*sizey]
  vertices+= [0.0]
  vertices+= [x-0.5*sizex]
  vertices+= [y-0.5*sizey]
  vertices+= [0.0]
  vertices+= [x-0.5*sizex]
  vertices+= [y+0.5*sizey]
  vertices+= [0.0]
  vertices+= [x]
  vertices+= [y]
  vertices+= [sizez]
  indices+= [idx+0]
  indices+= [idx+4]
  indices+= [idx+1]
  indices+= [idx+1]
  indices+= [idx+4]
  indices+= [idx+2]
  indices+= [idx+2]
  indices+= [idx+4]
  indices+= [idx+3]
  indices+= [idx+3]
  indices+= [idx+4]
  indices+= [idx+0]

#add_pyramid(0.0,0.0,0.2,0.2,0.05)
#add_pyramid(1.0,0.0,0.2,0.2,0.05)
#add_pyramid(2.0,0.0,0.2,0.2,0.05)

cstep= x_size/(cols-1)
rstep= y_size/(rows-1)
for c in range(0,cols):
  for r in range(0,rows):
    add_pyramid(cstep*c,rstep*r,pyrm_x,pyrm_y,pyrm_z)


print '// generator: '+ __file__
print '// x_size= '+str(x_size),
print '; y_size= '+str(y_size),
print '; pyrm_x= '+str(pyrm_x),
print '; pyrm_y= '+str(pyrm_y),
print '; pyrm_z= '+str(pyrm_z),
print '; cols= '+str(cols),
print '; rows= '+str(rows),
print ''

print 'Vertices=('
eol=','
for i in range(0,len(vertices)/3):
  if(i==len(vertices)/3-1):  eol=""
  print u"%f, %f, %f%s" % (vertices[3*i],vertices[3*i+1],vertices[3*i+2], eol)
print ')'

print 'Indices=('
eol=','
for i in range(0,len(indices)/3):
  if(i==len(indices)/3-1):  eol=""
  print u"%d, %d, %d%s" % (indices[3*i],indices[3*i+1],indices[3*i+2], eol)
print ')'
