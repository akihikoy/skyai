//-------------------------------------------------------------------------------------------
/*! \file    lc_pd.cpp
    \brief   libskyai - PC-controller  (source)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Feb.09, 2010-

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
#include <skyai/modules_std/lc_pd.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
using namespace std;
// using namespace boost;


//===========================================================================================
// class MLCSimplePD
//===========================================================================================

override void MLCSimplePD::slot_execute_action_exec (const TRealVector &a)
{
  target_= a;
  ltime_= 0.0l;
  is_active_= true;
}
//-------------------------------------------------------------------------------------------

override void MLCSimplePD::slot_start_time_step_exec (const TContinuousTime &time_step)
{
  if(!is_active_)  return;

  extract_pd_from_state (get_state(), state_proportional_, state_derivative_);
  LASSERT1op1(target_.length(),==,state_proportional_.length());
  LASSERT1op1(target_.length(),==,state_derivative_.length());
  LASSERT1op1(target_.length(),==,conf_.Kp.length());
  LASSERT1op1(target_.length(),==,conf_.Kd.length());
  const int dim= target_.length();
  command_.resize(dim);

  TypeExt<TRealVector>::const_iterator ikp(GenBegin(conf_.Kp));
  TypeExt<TRealVector>::const_iterator ikd(GenBegin(conf_.Kd));
  TypeExt<TRealVector>::const_iterator ix(GenBegin(state_proportional_));
  TypeExt<TRealVector>::const_iterator iv(GenBegin(state_derivative_));
  TypeExt<TRealVector>::const_iterator ir(GenBegin(target_));
  for(TypeExt<TRealVector>::iterator iu(GenBegin(command_)); iu!=GenEnd(command_);
      ++iu, ++ikp, ++ikd, ++ix, ++iv, ++ir)
  {
    (*iu)= (*ikp)*((*ir)-(*ix)) - (*ikd)*(*iv);
  }

  signal_execute_command.ExecAll(command_);
}
//-------------------------------------------------------------------------------------------

override void MLCSimplePD::slot_finish_time_step_exec (const TContinuousTime &time_step)
{
  if(!is_active_)  return;

  ltime_+=time_step;
  switch(conf_.EOACondition)
  {
    case eoacNone     : return;
    case eoacError    :
      is_active_= false;
      FIXME("conf_.EOACondition==eoacError is not implemented yet");
      break;
    case eoacInterval :
      if(ltime_+CONT_TIME_TOL>=conf_.Interval)
      {
        is_active_= false;
        signal_end_of_action.ExecAll();
      }
      break;
    default :
      LERROR("invalid conf_.EOACondition: "<<(int)conf_.EOACondition);
      lexit(df);
      break;
  }
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
SKYAI_ADD_MODULE(MLCSimplePD)
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
