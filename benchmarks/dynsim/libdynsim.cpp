//-------------------------------------------------------------------------------------------
/*! \file    libdynsim.cpp
    \brief   skyai - certain application (source)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Mar.28, 2013

    Copyright (C) 2013  Akihiko Yamaguchi

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
#include "libdynsim.h"
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{


/*virtual*/const TContinuousState& MRobotProbe::out_base_pose_get() const
{
  LASSERT1op1(base_index_,>=,0);
  GenResize(tmp_base_pose_,7);
  in_world.GetFirst().GetBodyPosition(base_index_,GenBegin(tmp_base_pose_),GenBegin(tmp_base_pose_)+3);
  in_world.GetFirst().GetBodyQuaternion(base_index_,GenBegin(tmp_base_pose_)+3,GenEnd(tmp_base_pose_));
  return tmp_base_pose_;
}
/*virtual*/const TContinuousState& MRobotProbe::out_base_vel_get() const
{
  LASSERT1op1(base_index_,>=,0);
  GenResize(tmp_base_vel_,6);
  in_world.GetFirst().GetBodyLinearVel(base_index_,GenBegin(tmp_base_vel_),GenBegin(tmp_base_vel_)+3);
  in_world.GetFirst().GetBodyAngularVel(base_index_,GenBegin(tmp_base_vel_)+3,GenEnd(tmp_base_vel_));
  return tmp_base_vel_;
}
/*virtual*/const TRealMatrix& MRobotProbe::out_base_rot_get() const
{
  LASSERT1op1(base_index_,>=,0);
  tmp_base_rot_.resize(3,3);
  in_world.GetFirst().GetBodyRotation(base_index_,GenBegin(tmp_base_rot_),GenEnd(tmp_base_rot_));
  return tmp_base_rot_;
}
//-------------------------------------------------------------------------------------------

/*virtual*/const TContinuousState& MRobotProbe::out_joint_angle_get() const
{
  LASSERT1op1(robot_index_,>=,0);
  in_world.GetFirst().GetJointAngles(robot_index_, tmp_joint_angle_);
  return tmp_joint_angle_;
}
/*virtual*/const TContinuousState& MRobotProbe::out_joint_vel_get() const
{
  LASSERT1op1(robot_index_,>=,0);
  in_world.GetFirst().GetJointAngVels(robot_index_, tmp_joint_vel_);
  return tmp_joint_vel_;
}
//-------------------------------------------------------------------------------------------

/*virtual*/const TContinuousState& MRobotProbe::out_force_get() const
{
  LASSERT1op1(robot_index_,>=,0);
  in_world.GetFirst().GetForces(robot_index_, tmp_force_);
  return tmp_force_;
}
//-------------------------------------------------------------------------------------------

/*virtual*/const TBoolVector& MRobotProbe::out_contact_raw_get() const
{
  LASSERT1op1(robot_index_,>=,0);
  const std::vector<bool> &contacts= in_world.GetFirst().LinkContacts(robot_index_);
  tmp_contact_raw_= contacts;
  return tmp_contact_raw_;
}
//-------------------------------------------------------------------------------------------

/*virtual*/const TBoolVector& MRobotProbe::out_contact_get() const
{
  LASSERT(contact_LPF_.isInitialized());
  const std::vector<float>  &lpf(contact_LPF_());
  tmp_contact_filtered_.resize(lpf.size());
  std::vector<float>::const_iterator lpf_itr(lpf.begin());
  for (std::vector<bool>::iterator res_itr(tmp_contact_filtered_.begin()),res_last(tmp_contact_filtered_.end()); res_itr!=res_last; ++res_itr,++lpf_itr)
    (*res_itr)= (*lpf_itr > conf_.BodyContactThreshold);
  return tmp_contact_filtered_;
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
SKYAI_ADD_MODULE(MDynamicsSimulator)
SKYAI_ADD_MODULE(MRobotProbe)
SKYAI_ADD_MODULE(MDynSimUnivTask)
//-------------------------------------------------------------------------------------------

MDynamicsSimulator *ptr_dynamics_simulator(NULL);

void ODEDS_DSStart()
{
  ptr_dynamics_simulator->Start();
}
void ODEDS_DSStop()
{
  ptr_dynamics_simulator->Stop();
}
void ODEDS_DSKeyEvent (int cmd)
{
  ptr_dynamics_simulator->KeyEvent(cmd);
}
void ODEDS_DSStep (int ds_pause)
{
  ptr_dynamics_simulator->Step(ds_pause);
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of loco_rabbits
//-------------------------------------------------------------------------------------------
