//-------------------------------------------------------------------------------------------
/*! \file    ode.h
    \brief   liblora - ODE extension  (header)
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
#ifndef loco_rabbits_ode_h
#define loco_rabbits_ode_h
//-------------------------------------------------------------------------------------------
#ifndef dDOUBLE
#define dDOUBLE
#endif
//-------------------------------------------------------------------------------------------
#include <lora/common.h>
#include <lora/variable_space_fwd.h>
#include <string>
#include <ode/ode.h>
#include <lora/stl_fwd.h>
//-------------------------------------------------------------------------------------------
#ifdef PACKAGE_BUGREPORT
  #undef PACKAGE_BUGREPORT
  #undef PACKAGE_NAME
  #undef PACKAGE_STRING
  #undef PACKAGE_TARNAME
  #undef PACKAGE_VERSION
#endif
//-------------------------------------------------------------------------------------------
#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif
//-------------------------------------------------------------------------------------------
#ifndef ODE_MINOR_VERSION
  #error ODE_MINOR_VERSION should be set in compile
  #error   ex. -DODE_MINOR_VERSION=10
#endif
//-------------------------------------------------------------------------------------------
class dJointGroup;
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

inline void InitializeODE (void)
{
  #if ODE_MINOR_VERSION>=9
  # if ODE_MINOR_VERSION==9
    dInitODE();
  # elif ODE_MINOR_VERSION>=10
    dInitODE2(0);
  # endif
  #endif
}

inline void SetupODE (void)  // use in void start()
{
  #if ODE_MINOR_VERSION>=10
    dAllocateODEDataForThread(dAllocateMaskAll);
  #endif
}

inline void TerminateODE (void)
{
  #if ODE_MINOR_VERSION>=9
    dCloseODE();
  #endif
}
//-------------------------------------------------------------------------------------------

struct TODEAngle
{
  dReal angle1, angle2;
  TODEAngle (void) : angle1(0.0), angle2(0.0) {};
  TODEAngle (const dReal &_a1) : angle1(_a1), angle2(0.0) {};
  TODEAngle (const dReal &_a1, const dReal &_a2) : angle1(_a1), angle2(_a2) {};
};
typedef std::map<dJointID, TODEAngle>  TODEJointAngleMap;
//-------------------------------------------------------------------------------------------

//!\brief set joint angles connected to the base (ib)
//! \param [in]ib  id of the base body
void ODESetArticulatedBodyJointAngles (dBodyID ib, const TODEJointAngleMap &angle_map);
//!\brief set position (newpos) and rotation (newR) of the articulated body (ib)
void ODESetArticulatedBodyPosRotR (dBodyID ib, dVector3 newpos, dMatrix3 newR);
//!\brief set position (newpos) and rotation (newq) of the articulated body (ib)
void ODESetArticulatedBodyPosRotQ (dBodyID ib, dVector3 newpos, dQuaternion newq);
//-------------------------------------------------------------------------------------------

//! from line 34 of file ode/src/rotation.cpp.
#define _R(R,i,j) ((R)[(i)*4+(j)])

inline void ODERot2Omega (const dMatrix3 R, dVector3 w, const double &_eps=1.0e-6)
{
  double alpha= (_R(R,0,0)+_R(R,1,1)+_R(R,2,2) - 1.0) / 2.0;;

  if((alpha-1.0 < _eps) && (alpha-1.0 > -_eps))
  { std::fill(w,w+sizeof(dVector3)/sizeof(dReal),0.0); }
  else
  {
    double th = std::acos(alpha);
    double tmp= 0.5 * th / std::sin(th);
    w[0] = tmp * (_R(R,2,1) - _R(R,1,2));
    w[1] = tmp * (_R(R,0,2) - _R(R,2,0));
    w[2] = tmp * (_R(R,1,0) - _R(R,0,1));
  }
}
//-------------------------------------------------------------------------------------------
#undef _R

namespace var_space{
  void Register (dSurfaceParameters &x, TVariableMap &mmap);}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_ode_h
//-------------------------------------------------------------------------------------------
