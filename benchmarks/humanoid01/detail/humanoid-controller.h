//-------------------------------------------------------------------------------------------
/*! \file
    \brief  benchmarks - humanoid controller
    \author Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date   2008-

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

    -----------------------------------------------------------------------------------------

    \note    \c posrot means position and rotation.
    \note    \c prvel  means linear and angular velocity (velocity of position and rotation).
    \note    \c angvel means angular velocity.
    \note    \c kin-state  means (posrot,angular)
    \note    \c dyn-state  means (posrot,prvel) or (angular,angvel) or (posrot,angular, prvel,angvel)

    \todo   need to re-implement (very ad-hoc codes...)
    \todo   do not use 'extract_n' because it arises memory re-allocation which slows down (FIXME)
*/
//-------------------------------------------------------------------------------------------
#ifndef humanoid_controller_h
#define humanoid_controller_h
//-------------------------------------------------------------------------------------------
#define OBJ_CONTROLLER humanoid_controller
//-------------------------------------------------------------------------------------------
#include <lora/ctrl_tools.h>
#include "humanoid-manoi.h"
#include "sim-tools.h"
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------
extern const int JOINT_NUM;
//-------------------------------------------------------------------------------------------
namespace humanoid_controller
{
//-------------------------------------------------------------------------------------------

void INITIALIZE_DIMENSIONS (void);

enum TControlConstraintKind {
    cckUnspecified=-1,
    cckNone=0,
    cckStrictSymmetric,
    cckWideSymmetric,
    cckFBSymmetric,
    cckLowerBody,  /*use only lower body*/
    cckLowerBodyAutoFoot  /*use only lower body; feet are automatically controlled to be level to the graund*/};

struct _type_joint_inv_map
{
  int    i;  // joint index
  double f;  // factor
};

namespace cck_strict_symmetric
{
  static const int joint_dim (5);
  static const int joint_map [joint_dim]
    ={/*  0*/  1, /*=J 4*/
      /*  1*/  3, /*=J 6*/
      /*  2*/  8, /*=J13*/
      /*  3*/  9, /*=J14*/
      /*  4*/ 10};/*=J15*/
  static const _type_joint_inv_map  joint_inv_map [JOINT_NUM]
    ={{/*J 0*/ -1,  0.0},
      {/*J 1*/  0,  1.0},
      {/*J 2*/ -1,  0.0},
      {/*J 3*/  1,  1.0},
      {/*J 4*/  0,  1.0},
      {/*J 5*/ -1,  0.0},
      {/*J 6*/  1,  1.0},
      {/*J 7*/ -1,  0.0},
      {/*J 8*/  2,  1.0},
      {/*J 9*/  3,  1.0},
      {/*J10*/  4,  1.0},
      {/*J11*/ -1,  0.0},
      {/*J12*/ -1,  0.0},
      {/*J13*/  2,  1.0},
      {/*J14*/  3,  1.0},
      {/*J15*/  4,  1.0},
      {/*J16*/ -1,  0.0}};
};
namespace cck_wide_symmetric
{
  static const int joint_dim (7);
  static const int joint_map [joint_dim]
    ={/*  0*/  1, /*= J 4*/
      /*  1*/  2, /*=-J 5*/
      /*  2*/  3, /*= J 6*/
      /*  3*/  7, /*=-J12*/ /*=-J11*/ /*=J16*/
      /*  4*/  8, /*= J13*/
      /*  5*/  9, /*= J14*/
      /*  6*/ 10};/*= J15*/
  static const _type_joint_inv_map  joint_inv_map [JOINT_NUM]
    ={{/*J 0*/ -1,  1.0},
      {/*J 1*/  0,  1.0},
      {/*J 2*/  1,  1.0},
      {/*J 3*/  2,  1.0},
      {/*J 4*/  0,  1.0},
      {/*J 5*/  1, -1.0},
      {/*J 6*/  2,  1.0},
      {/*J 7*/  3,  1.0},
      {/*J 8*/  4,  1.0},
      {/*J 9*/  5,  1.0},
      {/*J10*/  6,  1.0},
      {/*J11*/  3, -1.0},
      {/*J12*/  3, -1.0},
      {/*J13*/  4,  1.0},
      {/*J14*/  5,  1.0},
      {/*J15*/  6,  1.0},
      {/*J16*/  3,  1.0}};
};
namespace cck_fb_symmetric
{
  static const int joint_dim (6);
  static const int joint_map [joint_dim]
    ={/*  0*/  1, /*= J 4*/
      /*  1*/  2, /*=-J 5*/
      /*  2*/  3, /*= J 6*/
      /*  3*/  7, /*=-J12*/ /*=-J11*/ /*=J16*/
      /*  4*/  8, /*= J13*/
      /*  5*/  9};/*= J14*/
      /*J10 = J8 + J9*/ /*= J15*/
  static const _type_joint_inv_map  joint_inv_map [JOINT_NUM]
    ={{/*J 0*/ -1,  1.0},
      {/*J 1*/  0,  1.0},
      {/*J 2*/  1,  1.0},
      {/*J 3*/  2,  1.0},
      {/*J 4*/  0,  1.0},
      {/*J 5*/  1, -1.0},
      {/*J 6*/  2,  1.0},
      {/*J 7*/  3,  1.0},
      {/*J 8*/  4,  1.0},
      {/*J 9*/  5,  1.0},
      {/*J10*/ -1,  0.0},
      {/*J11*/  3, -1.0},
      {/*J12*/  3, -1.0},
      {/*J13*/  4,  1.0},
      {/*J14*/  5,  1.0},
      {/*J15*/ -1,  0.0},
      {/*J16*/  3,  1.0}};
  inline double get_joint_angle (const ColumnVector x, int base, int j)
    {
      if (joint_inv_map[j].i>=0)
        return joint_inv_map[j].f * x(base+joint_inv_map[j].i);
      else if (j==10 || j==15)
        return -(x(base+4) + x(base+5));
      else  return 0.0;
    };
};
namespace cck_lower_body /*cckLowerBody*/
{
  static const int joint_dim (10);
  static const int joint_map [joint_dim]
    ={/*  0*/  7,    // left
      /*  1*/  8,
      /*  2*/  9,
      /*  3*/ 10,
      /*  4*/ 11,
      /*  5*/ 12,    // right
      /*  6*/ 13,
      /*  7*/ 14,
      /*  8*/ 15,
      /*  9*/ 16 };
  static const _type_joint_inv_map  joint_inv_map [JOINT_NUM]
    ={{/*J 0*/ -1,  1.0},
      {/*J 1*/ -1,  1.0},
      {/*J 2*/ -1,  1.0},
      {/*J 3*/ -1,  1.0},
      {/*J 4*/ -1,  1.0},
      {/*J 5*/ -1,  1.0},
      {/*J 6*/ -1,  1.0},
      {/*J 7*/  0,  1.0},
      {/*J 8*/  1,  1.0},
      {/*J 9*/  2,  1.0},
      {/*J10*/  3,  1.0},
      {/*J11*/  4,  1.0},
      {/*J12*/  5,  1.0},
      {/*J13*/  6,  1.0},
      {/*J14*/  7,  1.0},
      {/*J15*/  8,  1.0},
      {/*J16*/  9,  1.0}};
};
namespace cck_lower_body_auto_foot
{
  static const int joint_dim (6);
  static const int joint_map [joint_dim]
    ={/*  0*/  7,    // left
      /*  1*/  8,
      /*  2*/  9,
      /*  3*/ 12,    // right
      /*  4*/ 13,
      /*  5*/ 14 };
  static const _type_joint_inv_map  joint_inv_map [JOINT_NUM]
    ={{/*J 0*/ -1,  1.0},
      {/*J 1*/ -1,  1.0},
      {/*J 2*/ -1,  1.0},
      {/*J 3*/ -1,  1.0},
      {/*J 4*/ -1,  1.0},
      {/*J 5*/ -1,  1.0},
      {/*J 6*/ -1,  1.0},
      {/*J 7*/  0,  1.0},
      {/*J 8*/  1,  1.0},
      {/*J 9*/  2,  1.0},
      {/*J10*/ -1,  1.0},
      {/*J11*/ -1,  1.0},
      {/*J12*/  3,  1.0},
      {/*J13*/  4,  1.0},
      {/*J14*/  5,  1.0},
      {/*J15*/ -1,  1.0},
      {/*J16*/ -1,  1.0}};
};


struct THumanoidControllerCondition
{
  ColumnVector               PDGainKp, PDGainKd;  //!< gain parameters for lowlevel controller (PD)
  ColumnVector               TorqueMax;
  TControlConstraintKind     ControlConstraintKind;

  THumanoidControllerCondition (TControlConstraintKind cckind=cckUnspecified) :
      PDGainKp                 (JOINT_NUM,2.5l),
      PDGainKd                 (JOINT_NUM,0.08l),
      TorqueMax               (JOINT_NUM, loco_rabbits::TorqueMax),
      ControlConstraintKind  (cckind)
    {
      INITIALIZE_DIMENSIONS();
    };
};
extern THumanoidControllerCondition hmdctrlcnd;
//-------------------------------------------------------------------------------------------

//===========================================================================================
// get state info
//===========================================================================================
int CTRL_JOINT_NUM   ;//!< number of controlled joints
int JOINT_STATE_DIM  ;//!< dim of joint-state (joint angle, joint angular-velocity)
int POSROT_DIM       ;
int PRVEL_DIM        ;
int BASE_STATE_DIM   ; //!< dim of base link state; x,y,z,[quaternion],vx,vy.vz,wx,wy,wz
int KIN_STATE_DIM    ;
int DYN_STATE_DIM    ;
int DYN_VEL_DIM      ;
int DYN_VEL_BASE     ;
int JOINT_ANGLE_BASE ;
int JOINT_VEL_BASE   ;
int WHOLE_JOINT_NUM        ;//!< number of whole joints
int WHOLE_JOINT_STATE_DIM  ;//!< dim of whole joints state

void INITIALIZE_DIMENSIONS (void)
{
  if (hmdctrlcnd.ControlConstraintKind==cckUnspecified)  return;
  switch (hmdctrlcnd.ControlConstraintKind)
  {
    case cckNone  :
      CTRL_JOINT_NUM   = (JOINT_NUM);
      POSROT_DIM       = (7);
      PRVEL_DIM        = (6);
      break;
    case cckStrictSymmetric  :
      CTRL_JOINT_NUM   = (cck_strict_symmetric::joint_dim);
      POSROT_DIM       = (5);  // x,y are eliminated
      PRVEL_DIM        = (6);
      break;
    case cckWideSymmetric  :
      CTRL_JOINT_NUM   = (cck_wide_symmetric::joint_dim);
      POSROT_DIM       = (5);  // x,y are eliminated
      PRVEL_DIM        = (6);
      break;
    case cckFBSymmetric  :
      CTRL_JOINT_NUM   = (cck_fb_symmetric::joint_dim);
      POSROT_DIM       = (5);  // x,y are eliminated
      PRVEL_DIM        = (6);
      break;
    case cckLowerBody    :
      CTRL_JOINT_NUM   = (cck_lower_body::joint_dim);
      POSROT_DIM       = (5);  // x,y are eliminated
      PRVEL_DIM        = (6);
      break;
    case cckLowerBodyAutoFoot    :
      CTRL_JOINT_NUM   = (cck_lower_body_auto_foot::joint_dim);
      POSROT_DIM       = (5);  // x,y are eliminated
      PRVEL_DIM        = (6);
      break;
    default  :
      LERROR("fatal: invalid hmdctrlcnd.ControlConstraintKind= "<<hmdctrlcnd.ControlConstraintKind);
      exit(1);
  }
  JOINT_STATE_DIM  = (CTRL_JOINT_NUM*2);  //!< dim of joint-state (joint angle, joint angular-velocity)
  BASE_STATE_DIM   = (POSROT_DIM + PRVEL_DIM); //!< dim of base link state; x,y,z,[quaternion],vx,vy.vz,wx,wy,wz
  #ifdef FLOATING_BASE
    KIN_STATE_DIM    = (POSROT_DIM     + CTRL_JOINT_NUM);
    DYN_STATE_DIM    = (BASE_STATE_DIM + JOINT_STATE_DIM);
    DYN_VEL_DIM      = (PRVEL_DIM      + CTRL_JOINT_NUM);
    DYN_VEL_BASE     = (POSROT_DIM     + CTRL_JOINT_NUM);
    JOINT_ANGLE_BASE = (POSROT_DIM);
    JOINT_VEL_BASE   = (DYN_VEL_BASE   + PRVEL_DIM);
  #else
    KIN_STATE_DIM    = (CTRL_JOINT_NUM);
    DYN_STATE_DIM    = (JOINT_STATE_DIM);
    DYN_VEL_DIM      = (CTRL_JOINT_NUM);
    DYN_VEL_BASE     = (CTRL_JOINT_NUM);
    JOINT_ANGLE_BASE = (0);
    JOINT_VEL_BASE   = (DYN_VEL_BASE);
  #endif
  WHOLE_JOINT_NUM    = (JOINT_NUM);
  WHOLE_JOINT_STATE_DIM = (WHOLE_JOINT_NUM*2);
}
//-------------------------------------------------------------------------------------------
inline int ctrlIndexToJointIndex (int ctrl_index)
{
  switch (hmdctrlcnd.ControlConstraintKind)
  {
  case cckNone              : return ctrl_index;
  case cckStrictSymmetric   : return cck_strict_symmetric::joint_map[ctrl_index];
  case cckWideSymmetric     : return cck_wide_symmetric::joint_map[ctrl_index];
  case cckFBSymmetric       : return cck_fb_symmetric::joint_map[ctrl_index];
  case cckLowerBody         : return cck_lower_body::joint_map[ctrl_index];
  case cckLowerBodyAutoFoot : return cck_lower_body_auto_foot::joint_map[ctrl_index];
  default  :
    LERROR("fatal: invalid hmdctrlcnd.ControlConstraintKind= "<<hmdctrlcnd.ControlConstraintKind);
    exit(1);
  }
  return -1;
}
//-------------------------------------------------------------------------------------------
// #define _st1(_j)  state1[_j]  // using array
#define _st1(_j)  state1(_j)  // using ColumnVector
#define _st2(_j)  state2(_j)  // using ColumnVector
#define DECL_STATE1  ColumnVector &state1
#define DECL_STATE2  ColumnVector &state2
inline void _get_whole_joint_angle (DECL_STATE1,int start)
{
  for (int j(0);j<WHOLE_JOINT_NUM;++j)
    _st1(start+j) = joint[j].getAngle();
}
inline void _get_whole_joint_angvel (DECL_STATE1,int start)
{
  for (int j(0);j<WHOLE_JOINT_NUM;++j)
    _st1(start+j) = joint[j].getAngleRate();
}
inline void getWholeJointState (DECL_STATE1)
{
  _get_whole_joint_angle(state1,0);
  _get_whole_joint_angvel(state1,WHOLE_JOINT_NUM);
}
inline ColumnVector getWholeJointState (void)
{
  ColumnVector state(WHOLE_JOINT_STATE_DIM);
  getWholeJointState(state);
  return state;
}

inline void _get_joint_angle (DECL_STATE1,int start)
{
  switch (hmdctrlcnd.ControlConstraintKind)
  {
  case cckNone  :
    for (int j(0);j<CTRL_JOINT_NUM;++j)
      _st1(start+j) = joint[j].getAngle();
    break;
  case cckStrictSymmetric  :
    for (int j(0);j<CTRL_JOINT_NUM;++j)
      _st1(start+j) = joint[cck_strict_symmetric::joint_map[j]].getAngle();
    break;
  case cckWideSymmetric  :
    for (int j(0);j<CTRL_JOINT_NUM;++j)
      _st1(start+j) = joint[cck_wide_symmetric::joint_map[j]].getAngle();
    break;
  case cckFBSymmetric  :
    for (int j(0);j<CTRL_JOINT_NUM;++j)
      _st1(start+j) = joint[cck_fb_symmetric::joint_map[j]].getAngle();
    break;
  case cckLowerBody    :
    for (int j(0);j<CTRL_JOINT_NUM;++j)
      _st1(start+j) = joint[cck_lower_body::joint_map[j]].getAngle();
    break;
  case cckLowerBodyAutoFoot   :
    for (int j(0);j<CTRL_JOINT_NUM;++j)
      _st1(start+j) = joint[cck_lower_body_auto_foot::joint_map[j]].getAngle();
    break;
  default  :
    LERROR("fatal: invalid hmdctrlcnd.ControlConstraintKind= "<<hmdctrlcnd.ControlConstraintKind);
    exit(1);
  }
}
inline void getJointAngle (DECL_STATE1,int start=0)
{
  _get_joint_angle (state1, start);
}

inline void _get_joint_angvel (DECL_STATE1,int start)
{
  switch (hmdctrlcnd.ControlConstraintKind)
  {
  case cckNone  :
    for (int j(0);j<CTRL_JOINT_NUM;++j)
      _st1(start+j) = joint[j].getAngleRate();
    break;
  case cckStrictSymmetric  :
    for (int j(0);j<CTRL_JOINT_NUM;++j)
      _st1(start+j) = joint[cck_strict_symmetric::joint_map[j]].getAngleRate();
    break;
  case cckWideSymmetric  :
    for (int j(0);j<CTRL_JOINT_NUM;++j)
      _st1(start+j) = joint[cck_wide_symmetric::joint_map[j]].getAngleRate();
    break;
  case cckFBSymmetric  :
    for (int j(0);j<CTRL_JOINT_NUM;++j)
      _st1(start+j) = joint[cck_fb_symmetric::joint_map[j]].getAngleRate();
    break;
  case cckLowerBody    :
    for (int j(0);j<CTRL_JOINT_NUM;++j)
      _st1(start+j) = joint[cck_lower_body::joint_map[j]].getAngleRate();
    break;
  case cckLowerBodyAutoFoot   :
    for (int j(0);j<CTRL_JOINT_NUM;++j)
      _st1(start+j) = joint[cck_lower_body_auto_foot::joint_map[j]].getAngleRate();
    break;
  default  :
    LERROR("fatal: invalid hmdctrlcnd.ControlConstraintKind= "<<hmdctrlcnd.ControlConstraintKind);
    exit(1);
  }
}

inline void getJointState (DECL_STATE1, int start=0)
{
  _get_joint_angle(state1,  start+ 0);
  _get_joint_angvel(state1, start+ CTRL_JOINT_NUM);
}

inline void _get_base_posrot (DECL_STATE1,int start)
{
  switch (hmdctrlcnd.ControlConstraintKind)
  {
  case cckNone  :
    _st1(start+0)  = body[0].getPosition()[0];  // x
    _st1(start+1)  = body[0].getPosition()[1];  // y
    _st1(start+2)  = body[0].getPosition()[2];  // z
    _st1(start+3)  = body[0].getQuaternion()[0]; // quaternion (w)
    _st1(start+4)  = body[0].getQuaternion()[1]; // quaternion (x)
    _st1(start+5)  = body[0].getQuaternion()[2]; // quaternion (y)
    _st1(start+6)  = body[0].getQuaternion()[3]; // quaternion (z)
    break;
  case cckStrictSymmetric  :
  case cckWideSymmetric  :
  case cckFBSymmetric  :
  case cckLowerBody  :
  case cckLowerBodyAutoFoot  :
    _st1(start+0)  = body[0].getPosition()[2];  // z
    _st1(start+1)  = body[0].getQuaternion()[0]; // quaternion (w)
    _st1(start+2)  = body[0].getQuaternion()[1]; // quaternion (x)
    _st1(start+3)  = body[0].getQuaternion()[2]; // quaternion (y)
    _st1(start+4)  = body[0].getQuaternion()[3]; // quaternion (z)
    break;
  default  :
    LERROR("fatal: invalid hmdctrlcnd.ControlConstraintKind= "<<hmdctrlcnd.ControlConstraintKind);
    exit(1);
  }
}
inline void _get_base_prvel (DECL_STATE1,int start)
{
  switch (hmdctrlcnd.ControlConstraintKind)
  {
  case cckNone  :
  case cckStrictSymmetric  :
  case cckWideSymmetric  :
  case cckFBSymmetric  :
  case cckLowerBody  :
  case cckLowerBodyAutoFoot  :
    _st1(start+0)  = body[0].getLinearVel()[0];  // vx
    _st1(start+1)  = body[0].getLinearVel()[1];  // vy
    _st1(start+2)  = body[0].getLinearVel()[2];  // vz
    _st1(start+3)  = body[0].getAngularVel()[0] ; // wx
    _st1(start+4)  = body[0].getAngularVel()[1] ; // wx
    _st1(start+5)  = body[0].getAngularVel()[2] ; // wx
    break;
  default  :
    LERROR("fatal: invalid hmdctrlcnd.ControlConstraintKind= "<<hmdctrlcnd.ControlConstraintKind);
    exit(1);
  }
}
inline void getBaseState (DECL_STATE1)
{
  _get_base_posrot(state1, 0);
  _get_base_prvel(state1, POSROT_DIM);
}

inline void getKinState (DECL_STATE1)
{
  #ifdef FLOATING_BASE
    _get_base_posrot  (state1, 0);
    _get_joint_angle  (state1, POSROT_DIM);
  #else
    _get_joint_angle  (state1, 0);
  #endif
}

inline void getDynState (DECL_STATE1, int start=0)
{
  #ifdef FLOATING_BASE
    _get_base_posrot  (state1, start+ 0);
    _get_joint_angle  (state1, start+ POSROT_DIM);
    _get_base_prvel   (state1, start+ DYN_VEL_BASE);
    _get_joint_angvel (state1, start+ JOINT_VEL_BASE);
  #else
    getJointState     (state1, start);
  #endif
}

//! \brief dynamic-state  -->  dynamic-vel
inline ColumnVector extractDynVel (const DECL_STATE1, int offset_1=0)
{
  return state1.extract_n(offset_1+DYN_VEL_BASE, DYN_VEL_DIM);
}
inline void extractDynVel (DECL_STATE2/*dst*/, const DECL_STATE1/*src*/, int offset_1=0)
{
  for(int i(0),j(DYN_VEL_BASE); i<DYN_VEL_DIM; ++i,++j)
    _st2(i) = _st1(j+offset_1);
}

//! \brief dynamic-state  -->  joint-state
inline ColumnVector extractJointState (const DECL_STATE1)
{
  ColumnVector res(JOINT_STATE_DIM);
  int i(0);
  for(int j(JOINT_ANGLE_BASE); i<CTRL_JOINT_NUM; ++i,++j)
    res(i) = _st1(j);
  for(int j(JOINT_VEL_BASE); i<JOINT_STATE_DIM; ++i,++j)
    res(i) = _st1(j);
  return res;
}
inline void extractJointState (DECL_STATE2/*dst*/, const DECL_STATE1/*src*/, int offset_1=0)
{
  //if (state2.length()==0)  state2.resize(JOINT_STATE_DIM);
  int i(0);
  for(int j(JOINT_ANGLE_BASE); i<CTRL_JOINT_NUM; ++i,++j)
    _st2(i) = _st1(j+offset_1);
  for(int j(JOINT_VEL_BASE); i<JOINT_STATE_DIM; ++i,++j)
    _st2(i) = _st1(j+offset_1);
}

//! \brief dynamic-state  -->  joint-angle
inline ColumnVector extractJointAngle (const DECL_STATE1)
{
  return state1.extract_n(JOINT_ANGLE_BASE, CTRL_JOINT_NUM);
}
inline void extractJointAngle (DECL_STATE2/*dst*/, const DECL_STATE1/*src*/, int offset_1=0)
{
  int i(0);
  for(int j(JOINT_ANGLE_BASE); i<CTRL_JOINT_NUM; ++i,++j)
    _st2(i) = _st1(j+offset_1);
}

//! \brief dynamic-state  -->  whole-joint-angle
inline ColumnVector extractWholeJointAngle (const DECL_STATE1)
{
  switch (hmdctrlcnd.ControlConstraintKind)
  {
  case cckNone  :
    return extractJointAngle (state1);
  case cckStrictSymmetric  :
    {
      ColumnVector state2 (JOINT_NUM,0.0);
      for (int j(0);j<JOINT_NUM;++j)
        if (cck_strict_symmetric::joint_inv_map[j].i>=0)
          _st2(j) = cck_strict_symmetric::joint_inv_map[j].f * _st1(JOINT_ANGLE_BASE+cck_strict_symmetric::joint_inv_map[j].i);
      return state2;
    }
  case cckWideSymmetric  :
    {
      ColumnVector state2 (JOINT_NUM,0.0);
      for (int j(0);j<JOINT_NUM;++j)
        if (cck_wide_symmetric::joint_inv_map[j].i>=0)
          _st2(j) = cck_wide_symmetric::joint_inv_map[j].f * _st1(JOINT_ANGLE_BASE+cck_wide_symmetric::joint_inv_map[j].i);
      return state2;
    }
  case cckFBSymmetric  :
    {
      ColumnVector state2 (JOINT_NUM,0.0);
      for (int j(0);j<JOINT_NUM;++j)
        _st2(j) = cck_fb_symmetric::get_joint_angle(state1,JOINT_ANGLE_BASE,j);
      return state2;
    }
  case cckLowerBody  :
    {
      ColumnVector state2 (JOINT_NUM,0.0);
      for (int j(0);j<JOINT_NUM;++j)
        if (cck_lower_body::joint_inv_map[j].i>=0)
          _st2(j) = cck_lower_body::joint_inv_map[j].f * _st1(JOINT_ANGLE_BASE+cck_lower_body::joint_inv_map[j].i);
      return state2;
    }
  case cckLowerBodyAutoFoot  :
    {
      ColumnVector state2 (JOINT_NUM,0.0);
      for (int j(0);j<JOINT_NUM;++j)
        if (cck_lower_body_auto_foot::joint_inv_map[j].i>=0)
          _st2(j) = cck_lower_body_auto_foot::joint_inv_map[j].f * _st1(JOINT_ANGLE_BASE+cck_lower_body_auto_foot::joint_inv_map[j].i);
      return state2;
    }
  default  :
    LERROR("fatal: invalid hmdctrlcnd.ControlConstraintKind= "<<hmdctrlcnd.ControlConstraintKind);
    exit(1);
  }
}

//! \brief dynamic-state  -->  kin-state
inline ColumnVector extractKinState (const DECL_STATE1)
{
  return state1.extract_n(0, KIN_STATE_DIM);
}

//! \brief dynamic-state  -->  base-pos, base-rot (quaternion)
inline void extractBasePosRot (const DECL_STATE1, dVector3 pos, dQuaternion rot)
{
  #ifdef FLOATING_BASE
    switch (hmdctrlcnd.ControlConstraintKind)
    {
    case cckNone  :
      pos[0] = _st1(0); // x
      pos[1] = _st1(1); // y
      pos[2] = _st1(2); // z
      rot[0] = _st1(3); // quaternion (w)
      rot[1] = _st1(4); // quaternion (x)
      rot[2] = _st1(5); // quaternion (y)
      rot[3] = _st1(6); // quaternion (z)
      break;
    case cckStrictSymmetric  :
    case cckWideSymmetric  :
    case cckFBSymmetric  :
    case cckLowerBody  :
    case cckLowerBodyAutoFoot  :
      pos[0] = 0.0; // x
      pos[1] = 0.0; // y
      pos[2] = _st1(0); // z
      rot[0] = _st1(1); // quaternion (w)
      rot[1] = _st1(2); // quaternion (x)
      rot[2] = _st1(3); // quaternion (y)
      rot[3] = _st1(4); // quaternion (z)
      break;
    default  :
      LERROR("fatal: invalid hmdctrlcnd.ControlConstraintKind= "<<hmdctrlcnd.ControlConstraintKind);
      exit(1);
    }
  #else
    LERROR("extractBasePosRot is valid only for FLOATING_BASE");
    exit(1);
  #endif
}

//! \brief real-torque(JOINT_NUM)  -->  control-input(CTRL_JOINT_NUM)
inline void extractCtrlInput (DECL_STATE2/*dst*/, const DECL_STATE1/*src*/)
{
  switch (hmdctrlcnd.ControlConstraintKind)
  {
  case cckNone  :
    for (int j(0);j<CTRL_JOINT_NUM;++j)
      _st2(j) = _st1(j);
    break;
  case cckStrictSymmetric  :
    for (int j(0);j<CTRL_JOINT_NUM;++j)
      _st2(j) = _st1(cck_strict_symmetric::joint_map[j]);
    break;
  case cckWideSymmetric  :
    for (int j(0);j<CTRL_JOINT_NUM;++j)
      _st2(j) = _st1(cck_wide_symmetric::joint_map[j]);
    break;
  case cckFBSymmetric  :
    for (int j(0);j<CTRL_JOINT_NUM;++j)
      _st2(j) = _st1(cck_fb_symmetric::joint_map[j]);
    break;
  case cckLowerBody  :
    for (int j(0);j<CTRL_JOINT_NUM;++j)
      _st2(j) = _st1(cck_lower_body::joint_map[j]);
    break;
  case cckLowerBodyAutoFoot  :
    for (int j(0);j<CTRL_JOINT_NUM;++j)
      _st2(j) = _st1(cck_lower_body_auto_foot::joint_map[j]);
    break;
  default  :
    LERROR("fatal: invalid hmdctrlcnd.ControlConstraintKind= "<<hmdctrlcnd.ControlConstraintKind);
    exit(1);
  }
}
//-------------------------------------------------------------------------------------------
#undef _st1
#undef _st2
#undef DECL_STATE1
#undef DECL_STATE2
//-------------------------------------------------------------------------------------------


//===========================================================================================
// SENSORS FOR CONTROL
//===========================================================================================

//! from line 34 of file ode/src/rotation.cpp.
#define _R(R,i,j) ((R)[(i)*4+(j)])

//!\TODO condier the singular poses!
inline TReal getRoll (int index)  //! return roll in radian
{
  // return real_fabs(real_atan2(_R(body[index].getRotation(),2,1), _R(body[index].getRotation(),2,2)));
  TReal sign22 = Sign(_R(body[index].getRotation(),2,2));
  TReal roll = real_atan2(sign22*_R(body[index].getRotation(),2,1), sign22*_R(body[index].getRotation(),2,2));
  // roll = real_fabs(roll);
  return roll;
}
inline TReal getRollAngVel (int index)  //! return angular velocity of roll in radian/sec
{
  return body[index].getAngularVel()[0];
}

//!\TODO condier the singular poses!
inline TReal getPitch (int index)  //! return pitch in radian
{
  LWARNING("getPitch looks incorrect. fix.");
  return real_atan2(_R(body[index].getRotation(),0,2), _R(body[index].getRotation(),2,2));
}
inline TReal getPitchAngVel (int index)  //! return angular velocity of pitch in radian/sec
{
  return body[index].getAngularVel()[1];
}

//!\TODO condier the singular poses!
inline TReal getYaw (int index)  //! return yaw in radian
{
  return real_atan2(_R(body[index].getRotation(),1,0), _R(body[index].getRotation(),0,0));
}
inline TReal getYawAngVel (int index)  //! return angular velocity of yaw in radian/sec
{
  return body[index].getAngularVel()[2];
}

//! return how the body is near to 'faceup' with [-1,1] (inner prod of ex and (0,0,1); 1 is faceup)
inline TReal faceupRatio (void)
{
  return _R(body[0].getRotation(),2,0);
}
//! return how the body is near to 'facedown' with [-1,1] (inner prod of ex and (0,0,-1); 1 is facedown)
inline TReal facedownRatio (void)
{
  return -1.0*_R(body[0].getRotation(),2,0);
}

#undef _R


//===========================================================================================
// lowlevel controller
//===========================================================================================

// state --> controlled joint state
inline void extractControlState (ColumnVector &q, const ColumnVector &x, int offset_x=0)
{
  if (q.length()==0)  q.resize(JOINT_STATE_DIM);
  extractJointState(q,x,offset_x);
}
inline void extractControlPos (ColumnVector &q, const ColumnVector &x, int offset_x=0)
{
  if (q.length()==0)  q.resize(CTRL_JOINT_NUM);
  extractJointAngle(q,x,offset_x);
}
//-------------------------------------------------------------------------------------------

// controlled joint angles --> whole joint angles
inline void controlTargetToJointTarget (const ColumnVector &c, ColumnVector &t)
{
  const int  nt (t.length());
  switch (hmdctrlcnd.ControlConstraintKind)
  {
  case cckNone  :
    t= c;
    break;
  case cckStrictSymmetric  :
    for(int j(0); j<nt; ++j)
      if (cck_strict_symmetric::joint_inv_map[j].i>=0)
            t(j)= cck_strict_symmetric::joint_inv_map[j].f * c(cck_strict_symmetric::joint_inv_map[j].i);
      else  t(j)= 0.0;
    break;
  case cckWideSymmetric  :
    for(int j(0); j<nt; ++j)
      if (cck_wide_symmetric::joint_inv_map[j].i>=0)
            t(j)= cck_wide_symmetric::joint_inv_map[j].f * c(cck_wide_symmetric::joint_inv_map[j].i);
      else  t(j)= 0.0;
    break;
  case cckFBSymmetric  :
    for(int j(0); j<nt; ++j)
      t(j) = cck_fb_symmetric::get_joint_angle(c,0,j);
    break;
  case cckLowerBody  :
    for(int j(0); j<nt; ++j)
      if (cck_lower_body::joint_inv_map[j].i>=0)
            t(j)= cck_lower_body::joint_inv_map[j].f * c(cck_lower_body::joint_inv_map[j].i);
      else  t(j)= 0.0;
    break;
  case cckLowerBodyAutoFoot  :
    for(int j(0); j<nt; ++j)
      if (cck_lower_body_auto_foot::joint_inv_map[j].i>=0)
            t(j)= cck_lower_body_auto_foot::joint_inv_map[j].f * c(cck_lower_body_auto_foot::joint_inv_map[j].i);
      else  t(j)= 0.0;
    break;
  default  :
    LERROR("fatal: invalid hmdctrlcnd.ControlConstraintKind= "<<hmdctrlcnd.ControlConstraintKind);
    exit(1);
  }
}
//-------------------------------------------------------------------------------------------

inline dReal get_angle_around_axis (const dMatrix3 R, const dVector3 a)
{
#define _R(R,i,j) ((R)[(i)*4+(j)])
  dVector3 rz2, ez2, ez2xa;
  dReal f= _R(R,0,2)*a[0]+_R(R,1,2)*a[1]+_R(R,2,2)*a[2];
  rz2[0]= _R(R,0,2)-f*a[0];
  rz2[1]= _R(R,1,2)-f*a[1];
  rz2[2]= _R(R,2,2)-f*a[2];
  dNormalize3(rz2);
  f= a[2];
  ez2[0]= 0.0-f*a[0];
  ez2[1]= 0.0-f*a[1];
  ez2[2]= 1.0-f*a[2];
  dNormalize3(ez2);
  dCROSS(ez2xa,=,ez2,a);
  return -std::atan2(dDOT(rz2,ez2xa), dDOT(rz2,ez2));
#undef _R
}
//-------------------------------------------------------------------------------------------

// target joint angle --> control torque
inline void lowLevelRobotModel (const ColumnVector &c_target, ColumnVector &input, ColumnVector jstate)
{
  static TPDController  PDC(JOINT_NUM, 0.0, 0.0, 0.0);
  static int init(-1);
  if (SimulationInit(init))
  {
    PDC.Kp = hmdctrlcnd.PDGainKp;
    PDC.Kd = hmdctrlcnd.PDGainKd;
    PDC.UMax = hmdctrlcnd.TorqueMax;
  }
  static ColumnVector q_target(JOINT_NUM,0.0);
  controlTargetToJointTarget (c_target, q_target);
  if (hmdctrlcnd.ControlConstraintKind == cckLowerBodyAutoFoot)
  {
    static int initL(-1);
    if (SimulationInit(initL)/* || bodies_contact_with_ground(LFOOT_INDEX,0.001f)*/)
    {
      q_target(LFOOT_JOINT_PITCH)  = 0.0;
      q_target(LFOOT_JOINT_ROLL)   = 0.0;
    }
    else
    {
      dVector3 a;
      const dReal *R(body[LFOOT_INDEX].getRotation());
      const dReal *w(body[LFOOT_INDEX].getAngularVel());
      dReal th[2], dth[2];
      joint[LFOOT_JOINT_PITCH].getAxis(a);
      th[0]= get_angle_around_axis(R,a);
      dth[0]= w[0]*a[0]+w[1]*a[1]+w[2]*a[2];
      joint[LFOOT_JOINT_ROLL].getAxis(a);
      th[1]= get_angle_around_axis(R,a);
      dth[1]= w[0]*a[0]+w[1]*a[1]+w[2]*a[2];
      // robot.joint(0).jointAsHinge().addTorque(kp*(target_angle[0]-th[0]) - kd*dth[0]);
      // robot.joint(1).jointAsHinge().addTorque(kp*(target_angle[1]-th[1]) - kd*dth[1]);
      jstate(LFOOT_JOINT_PITCH)  = -th[0];
      jstate(LFOOT_JOINT_ROLL)   = -th[1];
      jstate(JOINT_NUM+LFOOT_JOINT_PITCH)  = -dth[0];
      jstate(JOINT_NUM+LFOOT_JOINT_ROLL)   = -dth[1];
      q_target(LFOOT_JOINT_PITCH)  = 0.0;
      q_target(LFOOT_JOINT_ROLL)   = 0.0;
      // LDEBUG(th[0]<<"\t"<<th[1]);
    }

    static int initR(-1);
    if (SimulationInit(initR)/* || bodies_contact_with_ground(RFOOT_INDEX,0.001f)*/)
    {
      q_target(RFOOT_JOINT_PITCH)  = 0.0;
      q_target(RFOOT_JOINT_ROLL)   = 0.0;
    }
    else
    {
      dVector3 a;
      const dReal *R(body[RFOOT_INDEX].getRotation());
      const dReal *w(body[RFOOT_INDEX].getAngularVel());
      dReal th[2], dth[2];
      joint[RFOOT_JOINT_PITCH].getAxis(a);
      th[0]= get_angle_around_axis(R,a);
      dth[0]= w[0]*a[0]+w[1]*a[1]+w[2]*a[2];
      joint[RFOOT_JOINT_ROLL].getAxis(a);
      th[1]= get_angle_around_axis(R,a);
      dth[1]= w[0]*a[0]+w[1]*a[1]+w[2]*a[2];
      // robot.joint(0).jointAsHinge().addTorque(kp*(target_angle[0]-th[0]) - kd*dth[0]);
      // robot.joint(1).jointAsHinge().addTorque(kp*(target_angle[1]-th[1]) - kd*dth[1]);
      jstate(RFOOT_JOINT_PITCH)  = -th[0];
      jstate(RFOOT_JOINT_ROLL)   = -th[1];
      jstate(JOINT_NUM+RFOOT_JOINT_PITCH)  = -dth[0];
      jstate(JOINT_NUM+RFOOT_JOINT_ROLL)   = -dth[1];
      q_target(RFOOT_JOINT_PITCH)  = 0.0;
      q_target(RFOOT_JOINT_ROLL)   = 0.0;
      // LDEBUG(th[0]<<"\t"<<th[1]);
    }
  }
  input = PDC(jstate, q_target);
}
inline void lowLevelRobotModel (const ColumnVector &c_target, ColumnVector &input)
{
  static ColumnVector jstate(WHOLE_JOINT_STATE_DIM,0.0);
  jstate.resize(WHOLE_JOINT_STATE_DIM);
  getWholeJointState(jstate);
  lowLevelRobotModel (c_target, input, jstate);
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
};  // end of namespace humanoid_controller
//-------------------------------------------------------------------------------------------
};  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // humanoid_controller_h
//-------------------------------------------------------------------------------------------
