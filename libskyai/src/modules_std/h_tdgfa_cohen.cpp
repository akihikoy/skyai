//-------------------------------------------------------------------------------------------
/*! \file    h_tdgfa_cohen.cpp
    \brief   libskyai - Cohen's hierarchical RL (HRL) implementation
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Feb.22, 2010-

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
#include <skyai/modules_std/h_tdgfa_cohen.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
namespace tdgfa_tmpl_detail
{
using namespace std;
// using namespace boost;


//===========================================================================================
// template <typename t_action>
// class MHierarchicalTDGenericFACohen
//===========================================================================================

#define TEMPLATE_DEC  template <typename t_action>
#define XMODULE       MHierarchicalTDGenericFACohen <t_action>
#define XMODULE_STR  "MHierarchicalTDGenericFACohen <t_action>"

TEMPLATE_DEC
override void XMODULE::slot_start_episode_exec (void)
{
  higher_action_is_active_= false;
  signal_reset_lower.ExecAll();

  // TParent::slot_start_episode_exec();
  return_in_episode_= 0.0l;
  actions_in_episode_= 0;
  current_action_value_= 0.0l;
  reset_episode();

  if (slot_execute_higher_action.ConnectionSize()==0)  // root node
  {
    prepare_next_action();
    is_end_of_episode_= false;
    is_active_= true;
    current_action_value_= nextQ;
    signal_execute_action.ExecAll (select_action());
  }
}
//-------------------------------------------------------------------------------------------

TEMPLATE_DEC
override void XMODULE::slot_finish_action_exec (void)
{
  if (slot_update.ConnectionSize()!=0)  // not root node
  {
    if (higher_action_is_active_)
    {
      higher_action_is_active_= false;
      signal_end_of_higher_action.ExecAll();
    }
  }
  else  // root node
  {
    if (is_active_)
    {
      /*! reward */
      TSingleReward reward (0.0l);
      if (is_updatable())  reward= get_reward();

      /*! select next action; since it is used in Sarsa */
      prepare_next_action ();

      if (!is_end_of_episode_)
      {
        current_action_value_= nextQ;
        signal_execute_action.ExecAll (next_action);
      }

      root_values_.resize(3);
      root_values_(0)= reward;
      root_values_(1)= nextQ;
      root_values_(2)= nextV;
// LDBGVAR(root_values_.transpose());

      // update (reward);
      // signal_update_lower.ExecAll (old_action, reward, nextQ, nextV);

      // select_action();
      // ++actions_in_episode_;

      // signal_end_of_action.ExecAll();

      slot_update_exec (root_values_);

      if (is_end_of_episode_)
      {
        ++mem_.EpisodeNumber;
        is_active_= false;
        signal_end_of_episode.ExecAll();
      }
    }
  }
}
//-------------------------------------------------------------------------------------------

TEMPLATE_DEC
/*virtual*/ void XMODULE::slot_execute_higher_action_exec (void)
{
  /*! select next action; since it is used in Sarsa */
  prepare_next_action ();
  // is_active_= true;
  higher_action_is_active_= true;

  current_action_value_= nextQ;
  signal_execute_action.ExecAll (next_action);
}
//-------------------------------------------------------------------------------------------

TEMPLATE_DEC
/*virtual*/ void XMODULE::slot_update_exec (const TRealVector &values)
{
  // note: values= (const TSingleReward &reward, const TValue &nextQroot, const TValue &nextVroot)
  TValue tmp_nextQ, tmp_nextV;
  std::swap (nextQ, tmp_nextQ);
  std::swap (nextV, tmp_nextV);
  TSingleReward reward= values(0);
  nextQ= values(1);
  nextV= values(2);

  update (reward);
  if (!is_updatable() && conf_.UsingEligibilityTrace)  // TEST; update eligibility even when the value function is not updated
  {
    TParent::activity_trace() *= (conf_.Gamma * conf_.Lambda);
    // if (conf_.UsingReplacingTrace)  get_avf_replacing_trace (activity_trace());
  }

  std::swap (nextQ, tmp_nextQ);
  std::swap (nextV, tmp_nextV);

  signal_update_lower.ExecAll (old_action, values);

  select_action();
  ++actions_in_episode_;

  signal_end_of_action.ExecAll();
}
//-------------------------------------------------------------------------------------------


#undef XMODULE_STR
#undef XMODULE
#undef TEMPLATE_DEC
//-------------------------------------------------------------------------------------------
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MHierarchicalTDGenericFACohen,TContinuousAction)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MHierarchicalTDGenericFACohen,TDiscreteAction)

SKYAI_ADD_MODULE(MHierarchicalTDGenericFACohen_TContinuousAction)
SKYAI_ADD_MODULE(MHierarchicalTDGenericFACohen_TDiscreteAction)
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // end of namespace tdgfa_tmpl_detail
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

