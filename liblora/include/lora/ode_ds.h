//-------------------------------------------------------------------------------------------
/*! \file    ode_ds.h
    \brief   liblora - ODE-drawstuff extension  (header)
    \author  Akihiko Yamaguchi
    \date    2008-

    Copyright (C) 2008, 2010  Akihiko Yamaguchi

    This file is part of SkyAI.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
//-------------------------------------------------------------------------------------------
#ifndef loco_rabbits_ode_ds_h
#define loco_rabbits_ode_ds_h
//-------------------------------------------------------------------------------------------
#include <lora/math.h>
//-------------------------------------------------------------------------------------------
#include <drawstuff/drawstuff.h>
//-------------------------------------------------------------------------------------------
#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif
//-------------------------------------------------------------------------------------------
// select correct drawing functions
#ifdef dDOUBLE
#ifndef dsDrawBox
  #define dsDrawBox dsDrawBoxD
  #define dsDrawSphere dsDrawSphereD
  #define dsDrawCylinder dsDrawCylinderD
  #define dsDrawCapsule dsDrawCapsuleD
  #define dsDrawConvex dsDrawConvexD
#endif
#endif
//-------------------------------------------------------------------------------------------
struct dSurfaceParameters;
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

inline int GetAutoViewpointModeCount (void)
{
  return 4;
}
inline void SetAutoViewpointMode (int mode, const double *targetpos)
{
  if (mode==0) return;
  if (mode > GetAutoViewpointModeCount())
    {LERROR("invalid auto-viewpoint-mode: "<<mode); return;}
  if (mode>0)
  {
    static float xyz[3], hpr[3];
    dsGetViewpoint (xyz,hpr);
    if (mode==1 || mode==2)
    {
      static float vxy[2]={0.0,0.0}, nextxy[2];
      int i=mode-1;
      nextxy[i]= xyz[i] + (0.02*(targetpos[i]-xyz[i])-0.002*vxy[i]);
      vxy[i]= nextxy[i]-xyz[i];
      xyz[i]= nextxy[i];
    }
    else if (mode==3)
    {
      static float vhp[2]={0.0,0.0}, nexthp[2], targethp[2];
      targethp[0]= std::atan2(targetpos[1]-xyz[1],
                  targetpos[0]-xyz[0])*180.0/M_PI;
      double r= std::sqrt(Square(targetpos[0]-xyz[0])
                  +Square(targetpos[1]-xyz[1]));
      targethp[1]= std::atan2(targetpos[2]-xyz[2],r)*180.0/M_PI;
      for(int i(0);i<2;++i)
      {
        nexthp[i]=hpr[i] + (0.02*(targethp[i]-hpr[i])-0.002*vhp[i]);
        vhp[i]= nexthp[i]-hpr[i];
        hpr[i]= nexthp[i];
      }
    }
    if (mode==4)
    {
      static float vxy[2]={0.0,0.0}, nextxy[2];
      for(int i(0);i<2;++i)
      {
        nextxy[i]= xyz[i] + (0.02*(targetpos[i]-xyz[i])-0.002*vxy[i]);
        vxy[i]= nextxy[i]-xyz[i];
        xyz[i]= nextxy[i];
      }
    }
    dsSetViewpoint (xyz,hpr);
  }
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_ode_ds_h
//-------------------------------------------------------------------------------------------
