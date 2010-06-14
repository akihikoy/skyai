//-------------------------------------------------------------------------------------------
/*! \file    as_bftrans.cpp
    \brief   libskyai - action space BFTrans (part of DCOB) (source)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Feb.04, 2010-

    Copyright (C) 2010  Akihiko Yamaguchi

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
#include <skyai/modules_std/as_bftrans.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
using namespace std;
// using namespace boost;


//===========================================================================================
// class MBFTrans
//===========================================================================================

override void MBFTrans::slot_initialize_exec (void)
{
  clear_trj_gen();

  switch (conf_.TrajectoryGeneratorType)
  {
    case tgtJerkMin : trj_gen_= new TJerkMinTrjGenerator(0); break;
    case tgtAccMin  : trj_gen_= new TAccMinTrjGenerator (0); break;
    case tgtAccMin2 : trj_gen_= new TAccMinTrjGenerator2(0); break;
    default : LERROR("invalid conf_.TrajectoryGeneratorType:"<<(int)conf_.TrajectoryGeneratorType);  lexit(df);
  }

  trj_conf_.DynamicParamUpdate    = false;
  trj_conf_.ZeroUnspecifiedState = conf_.ZeroUnspecifiedState;
  trj_gen_->SetCondition(trj_conf_);

  trj_gen_->Reset (conf_.ProportionalDim/*CTRL_JOINT_NUM*/);

  in_control_= false;
  interval_neighbor_= 0.0l;
}
//-------------------------------------------------------------------------------------------

override void MBFTrans::slot_execute_action_exec (const TAction &a)
{
  if (trj_gen_==NULL) {LERROR("MBFTrans::slot_initialize should be called"); lexit(df);}

  TReal interval_factor_= a(0);
  target_.resize (a.length()-1);
  std::copy (GenBegin(a)+1, GenEnd(a), GenBegin(target_));

  if (conf_.SetTargetByState)
  {
    extract_pd_from_state (target_, target_proportional_, target_derivative_, target_pd_);
    if (target_pd_.length()!=2*conf_.ProportionalDim)
      {LERROR("target_pd_.length()!=2*conf_.ProportionalDim");
        LDBGVAR(target_pd_.length()); LDBGVAR(conf_.ProportionalDim); lexit(df);}
    trj_gen_->SetTargetTf (target_pd_, 0.0l);
  }
  else
  {
    if (target_.length()!=conf_.ProportionalDim)
      {LERROR("target_.length()!=conf_.ProportionalDim");
        LDBGVAR(target_.length()); LDBGVAR(conf_.ProportionalDim); lexit(df);}
    target_proportional_= target_;
    trj_gen_->SetTargetTf (target_, 0.0l);
  }

  extract_pd_from_state (get_state(), state_proportional_, state_derivative_, state_pd_);

  TReal trj_interval= trj_gen_->CalcIntervalFromFactor (conf_.ProportionalDim, state_proportional_, target_proportional_, interval_factor_);
  trj_gen_->SetTrjInterval (trj_interval);
  // TrjGen.step (state_pd_,dt);

  if (conf_.AbbreviateTrajectory)
  {
    // inner product of get_feature and get_distance_to_nearest_bf
    const TReal d_n= InnerProd (GenBegin(get_feature()),GenEnd(get_feature()), GenBegin(get_distance_to_nearest_bf()));

    TReal d(0.0l);
    if (conf_.SetTargetByState)  d= GetNorm(target_ - get_state());
    else                         d= GetNorm(target_ - state_proportional_);

    if(d_n<d)  interval_neighbor_= d_n/d * trj_interval;
    else       interval_neighbor_= trj_interval;
  }

  in_control_= true;
}
//-------------------------------------------------------------------------------------------

override void MBFTrans::slot_start_time_step_exec (const TContinuousTime &time_step)
{
  if (!in_control_)  return;
  if (trj_gen_==NULL) {LERROR("MBFTrans::slot_initialize should be called"); lexit(df);}

  // NOTE: at first time in each action, state_pd_ is already calculated in slot_execute_action_exec
  extract_pd_from_state (get_state(), state_proportional_, state_derivative_, state_pd_);

  trj_gen_->Step (state_pd_, time_step);

  signal_execute_command.ExecAll (trj_gen_->GetQd());
}
//-------------------------------------------------------------------------------------------

override void MBFTrans::slot_finish_time_step_exec (const TContinuousTime &time_step)
{
  if (!in_control_)  return;
  if (trj_gen_==NULL) {LERROR("MBFTrans::slot_initialize should be called"); lexit(df);}

  if (!trj_gen_->IsInControl())
    in_control_= false;

  if (conf_.AbbreviateTrajectory)
  {
    interval_neighbor_-=time_step;
    if (interval_neighbor_<time_step)
      in_control_= false;
  }

  if (!in_control_)  signal_end_of_action.ExecAll();
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
SKYAI_ADD_MODULE(MBFTrans)
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
