#!/usr/bin/python
# -*- coding: utf-8 -*-
from random import random

#Parameters:
x_size= 10.0
y_size= 10.0
shp_rad= 1.0
z_offset= -0.9
col_r= 0.8
col_g= 0.8
col_b= 0.8
col_a= 1
cols= 15 # num of x-lines of the grid
rows= 15 # num of y-lines of the grid

print '// generator: '+ __file__
print '// x_size= '+str(x_size),
print '; y_size= '+str(y_size),
print '; shp_rad= '+str(shp_rad),
print '; z_offset= '+str(z_offset),
print '; col_r= '+str(col_r),
print '; col_g= '+str(col_g),
print '; col_b= '+str(col_b),
print '; col_a= '+str(col_a),
print '; cols= '+str(cols),
print '; rows= '+str(rows),
print ''

idx=0
cstep= x_size/(cols-1)
rstep= y_size/(rows-1)
for c in range(0,cols):
  for r in range(0,rows):
    x= -x_size/2.0+cstep*c
    y= -y_size/2.0+rstep*r
    print u"[%i]={Geometry={Type=\"gtSphere\"; Radius=%f; Color= (%f,%f,%f,%f);}; Position= (%f,%f,%f);}" % (idx,shp_rad,col_r,col_g,col_b,col_a,x,y,z_offset)
    idx+=1

