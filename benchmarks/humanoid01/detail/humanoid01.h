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
*/
//-------------------------------------------------------------------------------------------
#ifndef manoi01_h
#define manoi01_h
//-------------------------------------------------------------------------------------------
#include "humanoid-manoi.h"
#include "humanoid-controller.h"
#include "sim-tools.h"
#include "sim-objects.h"
//-------------------------------------------------------------------------------------------
// #include <csignal>  // for signal()
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

namespace ad_hoc
{
  dReal BaselinkLinearAcclX;
}

namespace humanoid_controller
{
  THumanoidControllerCondition   hmdctrlcnd(cckNone);
}
TSimulationCondition           simulationcnd;
//-------------------------------------------------------------------------------------------

inline bool fallenDown (void)
{
  return bodies_contact_with_ground(BASELINK_INDEX) || bodies_contact_with_ground(HEADLINK_INDEX);
}
//-------------------------------------------------------------------------------------------


inline int getStateDim (void)
{
  return humanoid_controller::DYN_STATE_DIM;
}
inline void getState (ColumnVector &state)
{
  state.resize(getStateDim());
  humanoid_controller::getDynState (state);
}
//-------------------------------------------------------------------------------------------

void initSimulation2 (bool using_cart=false)
{
  initSimulation();
  sim_objects::setup_world();
  if (using_cart)
    sim_objects::makeCar(0.8,0.0);

  ad_hoc::BaselinkLinearAcclX= 0.0;
}
//-------------------------------------------------------------------------------------------

void worldStep (const ColumnVector &input, const TReal &TimeStep, TReal &stepcost, int cheat_controller=0)
  //! take a simulation step
{
  using namespace humanoid_controller;
  using namespace std;
  static ColumnVector tq_input (JOINT_NUM,0.0);
  for (int j(0);j<JOINT_NUM;++j)
    tq_input(j) = input(j); //+ndrand();
  for (int j(0);j<JOINT_NUM;++j)
  {
    tq_input(j) = ApplyRange(tq_input(j),-hmdctrlcnd.TorqueMax(j),hmdctrlcnd.TorqueMax(j));
    joint[j].addTorque (tq_input(j));
  }

  TReal cheat_cost(0.0l);
  if (cheat_controller==0)
  {
    // do nothing
  }
  else if (cheat_controller==1)
  {
    /* body balancer */{
      const dReal Kp(10.0), Kd(2.0);
      const dReal Kp2(100.0), Kd2(20.0);
      const dReal th_angle(0.1*M_PI), th_pos(0.6*get_init_body_com_height());
      dVector3 omega;
      ODERot2Omega(body[0].getRotation(), omega);
      dReal tq;
      const TReal cheat_cost_f (1.0l);
      if (real_fabs(omega[0]) > th_angle)
      {
        tq= Kp*(-omega[0])-Kd*body[0].getAngularVel()[0];
        body[0].addTorque(tq, 0.0, 0.0);
        cheat_cost+= cheat_cost_f*Square(tq);
      }
      if (real_fabs(omega[1]) > th_angle)
      {
        tq= Kp*(-omega[1])-Kd*body[0].getAngularVel()[1];
        body[0].addTorque(0.0, tq, 0.0);
        cheat_cost+= cheat_cost_f*Square(tq);
      }
      if (body[0].getPosition()[2] < th_pos)
      {
        tq= Kp2*(get_init_body_com_height()-body[0].getPosition()[2])-Kd2*body[0].getLinearVel()[2];
        body[0].addForce(0.0, 0.0, tq);
        cheat_cost+= cheat_cost_f*Square(tq);
      }
    }//*/
  }
  else if (cheat_controller==2)
  {
    const dReal Kp(10.0), Kd(2.0);
    const dReal Kp2(100.0), Kd2(20.0);
    const dReal th_angle(0.0), th_pos(0.1);
    dVector3 omega;
    ODERot2Omega(body[0].getRotation(), omega);
    dReal tq;
    const TReal cheat_cost_f (0.01l);
    if (real_fabs(omega[0]) > th_angle)
    {
      tq= Kp*(-omega[0])-Kd*body[0].getAngularVel()[0];
      body[0].addTorque(tq, 0.0, 0.0);
      cheat_cost+= cheat_cost_f*Square(tq);
    }
    if (real_fabs(omega[1]) > th_angle)
    {
      tq= Kp*(-omega[1])-Kd*body[0].getAngularVel()[1];
      body[0].addTorque(0.0, tq, 0.0);
      cheat_cost+= cheat_cost_f*Square(tq);
    }
    if (real_fabs(omega[2]) > th_angle)
    {
      tq= Kp*(-omega[2])-Kd*body[0].getAngularVel()[2];
      body[0].addTorque(0.0, 0.0, tq);
      cheat_cost+= cheat_cost_f*Square(tq);
    }
    if (real_fabs(body[0].getPosition()[1]) > th_pos)
    {
      tq= Kp2*(-body[0].getPosition()[1])-Kd2*body[0].getLinearVel()[1];
      body[0].addForce(0.0, tq, 0.0);
      cheat_cost+= cheat_cost_f*Square(tq);
    }
  }
  else
  {
    LERROR("invalid cheat_controller: "<<cheat_controller);
    lexit(df);
  }

  dReal baselink_linear_velx= body[BASELINK_INDEX].getLinearVel()[0];

  stepSimulation (TimeStep);

  ad_hoc::BaselinkLinearAcclX= (body[BASELINK_INDEX].getLinearVel()[0]-baselink_linear_velx)/TimeStep;


  //! step-cost
  stepcost = (GetNorm(tq_input) + 10.0l*cheat_cost) * TimeStep;

  contactgroup.empty();
}
//-------------------------------------------------------------------------------------------

//===========================================================================================

inline TReal getGoalRewardJump6 (const TReal &dt, const TReal &init_head_height)
{
  const TReal  MIN_HEAD_HEIGHT_RATE(0.75l);
  TReal r(0.0l);
  if (bodies_contact_with_ground(LFOOT_INDEX) || bodies_contact_with_ground(RFOOT_INDEX))  return 0.0l;
    // not add to reward when the robot is touching to the ground
  if (body[HEADLINK_INDEX].getLinearVel()[2]<0.0l)  return 0.0l;
    // not add to reward when the head velocity is directed to down

  dReal head_h = body[HEADLINK_INDEX].getPosition()[2];
  if (head_h < MIN_HEAD_HEIGHT_RATE * init_head_height)  return 0.0l;
  r = 100.0l * (head_h) * dt;
  return r;
}

//! reward for forward moving task
inline TReal getGoalRewardMove3 (void)
{
  TReal r= body[BASELINK_INDEX].getLinearVel()[0];
  r= 0.01l*r - 0.1l*Square(body[BASELINK_INDEX].getPosition()[1]);
  return r;
}
//! reward for forward moving task
inline TReal getGoalRewardMove4 (void)
{
  TReal r= ad_hoc::BaselinkLinearAcclX;
  r= 0.001l*r - 0.1l*Square(body[BASELINK_INDEX].getPosition()[1]);
  return r;
}


//! reward for forward rolling task
inline TReal getGoalRewardForwardroll1 (const TReal &dt)
{
  bool contacting(false);
  for (int j(0); j<BODY_NUM; ++j)
    if (bodies_contact_with_ground(j))  contacting=true;
  TReal r= body[BASELINK_INDEX].getAngularVel()[1] * dt * 5000.0l;
  if (r>=0.0l && !contacting)  return 0.0l;  // not add reward when the robot is floating completely (except for r is negative)
  r= 0.01*r - 0.1*Square(body[BASELINK_INDEX].getPosition()[1]);
  return r;
}
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // manoi01_h
//-------------------------------------------------------------------------------------------
