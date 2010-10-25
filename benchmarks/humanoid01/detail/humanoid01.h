//-------------------------------------------------------------------------------------------
/*! \file    manoi01.h
    \brief   benchmarks - humanoid motion learning task
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Nov.10, 2009-

    Copyright (C) 2009, 2010  Akihiko Yamaguchi

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
*/
//-------------------------------------------------------------------------------------------
#ifndef manoi01_h
#define manoi01_h
//-------------------------------------------------------------------------------------------
#include "humanoid-manoi.h"
#include "sim-tools.h"
#include "sim-objects.h"
#include <lora/stl_math.h>
#include <lora/type_gen_oct.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

TSimulationCondition           simulationcnd;
//-------------------------------------------------------------------------------------------


//===========================================================================================
// get state info
//===========================================================================================

static const int POSROT_DIM  (7);
static const int PRVEL_DIM   (6);
//-------------------------------------------------------------------------------------------

inline void GetJointAngle (ColumnVector &state1)
{
  if(state1.length()<JOINT_NUM)  state1.resize(JOINT_NUM);
  for (int j(0);j<JOINT_NUM;++j)  state1(j) = joint[j].getAngle();
}

inline void GetJointAngVel (ColumnVector &state1,int start=0)
{
  if(state1.length()<JOINT_NUM)  state1.resize(JOINT_NUM);
  for (int j(0);j<JOINT_NUM;++j)  state1(j) = joint[j].getAngleRate();
}

inline void GetJointState (ColumnVector &state1)
{
  if(state1.length()<JOINT_NUM*2)  state1.resize(JOINT_NUM*2);
  for (int j(0);j<JOINT_NUM;++j)  state1(j) = joint[j].getAngle();
  for (int j(0);j<JOINT_NUM;++j)  state1(JOINT_NUM+j) = joint[j].getAngleRate();
}

inline void GetBasePosRot (ColumnVector &state1)
{
  if(state1.length()<POSROT_DIM)  state1.resize(POSROT_DIM);
  state1(0) = body[0].getPosition()[0];  // x
  state1(1) = body[0].getPosition()[1];  // y
  state1(2) = body[0].getPosition()[2];  // z
  state1(3) = body[0].getQuaternion()[0]; // quaternion (w)
  state1(4) = body[0].getQuaternion()[1]; // quaternion (x)
  state1(5) = body[0].getQuaternion()[2]; // quaternion (y)
  state1(6) = body[0].getQuaternion()[3]; // quaternion (z)
}
inline void GetBaseVel (ColumnVector &state1)
{
  if(state1.length()<PRVEL_DIM)  state1.resize(PRVEL_DIM);
  state1(0) = body[0].getLinearVel()[0];  // vx
  state1(1) = body[0].getLinearVel()[1];  // vy
  state1(2) = body[0].getLinearVel()[2];  // vz
  state1(3) = body[0].getAngularVel()[0] ; // wx
  state1(4) = body[0].getAngularVel()[1] ; // wy
  state1(5) = body[0].getAngularVel()[2] ; // wz
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
// additional state
//===========================================================================================

//! from ode/src/rotation.cpp l.34
#define _R(R,i,j) ((R)[(i)*4+(j)])

/*!\brief return roll in radian
    \todo TODO need to consider the singular postures */
inline TReal GetRoll (int index)
{
  // return real_fabs(real_atan2(_R(body[index].getRotation(),2,1), _R(body[index].getRotation(),2,2)));
  TReal sign22 = Sign(_R(body[index].getRotation(),2,2));
  TReal roll = real_atan2(sign22*_R(body[index].getRotation(),2,1), sign22*_R(body[index].getRotation(),2,2));
  // roll = real_fabs(roll);
  return roll;
}

/*!\brief return pitch in radian
    \todo TODO need to consider the singular postures
    \todo FIXME incorrect calculation  */
inline TReal GetPitch (int index)
{
  return real_atan2(_R(body[index].getRotation(),0,2), _R(body[index].getRotation(),2,2));
}

/*!\brief return yaw in radian
    \todo TODO need to consider the singular postures  */
inline TReal GetYaw (int index)
{
  return real_atan2(_R(body[index].getRotation(),1,0), _R(body[index].getRotation(),0,0));
}

/*!\brief return [-1,1] indicating how the body is near to the face-up posture.
    the return is the inner product of ex and (0,0,1). larger is nearer */
inline TReal GetFaceupRatio (void)
{
  return _R(body[0].getRotation(),2,0);
}
/*!\brief return [-1,1] indicating how the body is near to the face-down posture.
    the return is the inner product of ex and (0,0,-1). larger is nearer */
inline TReal GetFacedownRatio (void)
{
  return -1.0*_R(body[0].getRotation(),2,0);
}

//-------------------------------------------------------------------------------------------
#undef _R
//-------------------------------------------------------------------------------------------


//===========================================================================================
// execution routine
//===========================================================================================

void initSimulation2 (bool using_cart=false)
{
  initSimulation();
  sim_objects::setup_world();
  if (using_cart)
    sim_objects::makeCar(0.8,0.0);
}
//-------------------------------------------------------------------------------------------

void worldStep (const ColumnVector &input, const ColumnVector &torque_max, const TReal &TimeStep, TReal &stepcost)
  //! take a simulation step
{
  using namespace std;
  static ColumnVector tq_input (JOINT_NUM,0.0);
  for (int j(0);j<JOINT_NUM;++j)
    tq_input(j) = input(j); //+ndrand();
  for (int j(0);j<JOINT_NUM;++j)
  {
    tq_input(j) = ApplyRange(tq_input(j),-torque_max(j),torque_max(j));
    joint[j].addTorque (tq_input(j));
  }

  stepSimulation (TimeStep);

  //! step-cost
  stepcost = (GetNorm(tq_input)) * TimeStep;

  contactgroup.empty();
}
//-------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // manoi01_h
//-------------------------------------------------------------------------------------------
