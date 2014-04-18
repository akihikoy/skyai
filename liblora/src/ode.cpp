//-------------------------------------------------------------------------------------------
/*! \file    ode.cpp
    \brief   liblora - ODE extension  (source)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Nov.30, 2008-

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
#ifndef ODE_MINOR_VERSION
#  define ODE_MINOR_VERSION 10
#  warning ODE_MINOR_VERSION is assigned to 10
#endif
#include <lora/ode.h>
//-------------------------------------------------------------------------------------------
#include <lora/string.h>
#include <lora/string_list.h>
#include <lora/variable_space.h>
//-------------------------------------------------------------------------------------------
#include <map>
//-------------------------------------------------------------------------------------------
// #if ODE_MINOR_VERSION>=10
// #include <ode/joints/joint.h>
// #else
// #include <../ode/src/joint.h>
// #endif
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------
using namespace std;
// using namespace boost;
//-------------------------------------------------------------------------------------------


//===========================================================================================
// SET POSE/ROT/JOINT_ANGLE
//===========================================================================================

/*! \brief rotate links that are connected to ib
    \param  [in]pos  center of rotation
    \param  [in]rot  rotation matrix  */
static void i_rot_succeeding_bodies (dBodyID ib, dJointID parent, const dVector3 pos, const dMatrix3 rot)
{
  dVector3 c, d;
  //memcpy (c, body[i].getPosition()[], sizeof(dVector3));
  memcpy (d, dBodyGetPosition(ib), sizeof(dVector3));
  dOPE(d,-=,pos);
  dMULTIPLY0_331 (c,rot,d);  // c = rot * d
  dOPE(c,+=,pos);
  dMatrix3 R;
  dMULTIPLY0_333 (R,rot,dBodyGetRotation(ib));  // R = rotR * rotR(i)
  dBodySetPosition (ib,c[0],c[1],c[2]);
  dBodySetRotation (ib,R);

  for (int j(0); j<dBodyGetNumJoints(ib); ++j)
  {
    dJointID ij = dBodyGetJoint(ib,j);
    if(ij==parent) continue;
    dBodyID     ib2=dJointGetBody(ij,0);
    if(ib2==ib) ib2=dJointGetBody(ij,1);
    if(ib2==0) continue;  // Attached to the space
    if(ib2==ib) {LERROR("fatal!"); exit(1);}
    i_rot_succeeding_bodies(ib2, ij, pos, rot);
  }
}
//-------------------------------------------------------------------------------------------

/*! \brief move all succeeding bodies connected to ib in distance d */
static void i_move_succeeding_bodies (dBodyID ib, dJointID parent, const dVector3 d)
{
  dBodySetPosition (ib, dBodyGetPosition(ib)[0]+d[0], dBodyGetPosition(ib)[1]+d[1], dBodyGetPosition(ib)[2]+d[2]);
  // for each body connected to ib
  for (int j(0); j<dBodyGetNumJoints(ib); ++j)
  {
    dJointID ij = dBodyGetJoint(ib,j);
    if(ij==parent) continue;

    dBodyID     ib2=dJointGetBody(ij,0);
    if(ib2==ib) ib2=dJointGetBody(ij,1);
    if(ib2==0) continue;  // Attached to the space
    if(ib2==ib) {LERROR("fatal!"); exit(1);}
    i_move_succeeding_bodies(ib2, ij, d);
  }
}
//-------------------------------------------------------------------------------------------

//!\brief  set joint angles connected to the base (ib) except for the parent
static void i_set_joint_angles (dBodyID ib, dJointID parent, const TODEJointAngleMap &angle_map)
{
  if (ib==0)  return;
  for (int j(0); j<dBodyGetNumJoints(ib); ++j)
  {
    dJointID ij = dBodyGetJoint(ib,j);
    if(ij==parent) continue;

    dReal  sign (1.0);
    dBodyID     ib2=dJointGetBody(ij,0);
    if(ib2==ib) {ib2=dJointGetBody(ij,1); sign=-1.0;}
    if(ib2==0) continue;  // Attached to the space
    if(ib2==ib) {LERROR("fatal!"); exit(1);}

    TODEJointAngleMap::const_iterator itr_angle= angle_map.find(ij);
    if (itr_angle!=angle_map.end())
    {
      const TODEAngle &angle(itr_angle->second);
      if (dJointGetType(ij)==dJointTypeHinge)
      {
        // calculate p, rotR from (dReal)(dBodyGetData(ij))
        dVector3 a, p;
        dJointGetHingeAnchor(ij, p);
        dJointGetHingeAxis(ij, a);
        dMatrix3 rotR;
        dRFromAxisAndAngle (rotR,a[0],a[1],a[2],sign*angle.angle1);
        i_rot_succeeding_bodies(ib2, ij, p, rotR);
      }
      else if (dJointGetType(ij)==dJointTypeSlider)
      {
        dVector3 d;
        dJointGetSliderAxis(ij, d);
        dOPEC(d,*=,sign*angle.angle1);
        // LDEBUG(angle.angle1<<"  "<<d[0]<<" "<<d[1]<<" "<<d[2]);
        i_move_succeeding_bodies(ib2, ij, d);
      }
      else if (dJointGetType(ij)==dJointTypeUniversal)
      {
        // calculate p, rotR from (dReal)(dBodyGetData(ij))
        dVector3 a, p;
        dMatrix3 rotR;
        dJointGetUniversalAnchor(ij, p);
        dJointGetUniversalAxis1(ij, a);
        dRFromAxisAndAngle (rotR,a[0],a[1],a[2],sign*angle.angle1);
        i_rot_succeeding_bodies(ib2, ij, p, rotR);
        dJointGetUniversalAxis2(ij, a);
        dRFromAxisAndAngle (rotR,a[0],a[1],a[2],sign*angle.angle2);
        i_rot_succeeding_bodies(ib2, ij, p, rotR);
      }
      // else if (dJointGetType(ij)==dJointTypeHinge2)
      // {
      // }
      else
      {
        LWARNING("in i_set_joint_angles, not implemented joint type ("<<dJointGetType(ij)<<")");
      }
    }
    i_set_joint_angles(ib2, ij, angle_map);
  }
}
//-------------------------------------------------------------------------------------------

//!\brief set joint angles connected to the base (ib)
//! \param [in]ib  id of the base body
void ODESetArticulatedBodyJointAngles (dBodyID ib, const TODEJointAngleMap &angle_map)
{
  i_set_joint_angles(ib, NULL, angle_map);
}
//-------------------------------------------------------------------------------------------


//!\brief set position (newpos) and rotation (newR) of the articulated body (ib)
void ODESetArticulatedBodyPosRotR (dBodyID ib, dVector3 newpos, dMatrix3 newR)
{
  dVector3 p;       // current base-position
  memcpy (p, dBodyGetPosition(ib), sizeof(dVector3));
  i_rot_succeeding_bodies (ib, NULL, p, newR);
  dVector3 d = {
    newpos[0]-dBodyGetPosition(ib)[0],
    newpos[1]-dBodyGetPosition(ib)[1],
    newpos[2]-dBodyGetPosition(ib)[2]};  // FIXME: dBodyGetPosition(ib) is equal to p ??
  i_move_succeeding_bodies (ib, NULL, d);
}
//-------------------------------------------------------------------------------------------

//!\brief set position (newpos) and rotation (newq) of the articulated body (ib)
void ODESetArticulatedBodyPosRotQ (dBodyID ib, dVector3 newpos, dQuaternion newq)
{
  dMatrix3 newR;
  dNormalize4(newq);
  dQtoR (newq, newR);
  ODESetArticulatedBodyPosRotR (ib, newpos, newR);
}
//-------------------------------------------------------------------------------------------

//===========================================================================================



namespace var_space{
  void Register (dSurfaceParameters &x, TVariableMap &mmap)
  {
    #define ADD(x_member)  AddToVarMap(mmap, #x_member, x.x_member)
    ADD( mode          );
    ADD( mu            );
    ADD( mu2           );
    ADD( bounce        );
    ADD( bounce_vel    );
    ADD( soft_erp      );
    ADD( soft_cfm      );
    ADD( motion1       );
    ADD( motion2       );
    ADD( slip1         );
    ADD( slip2         );
    #undef ADD
  }
}
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
}
//-------------------------------------------------------------------------------------------
