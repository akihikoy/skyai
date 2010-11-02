//-------------------------------------------------------------------------------------------
/*! \file    tdgfa_tmpl.cpp
    \brief   libskyai - reinforcement learning module using temporal difference methods
              with a generic function approximator (template for state,action,parameters)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Dec.04, 2009-

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

    \note include this file to instantiate for specific types
*/
//-------------------------------------------------------------------------------------------
#ifndef skyai_tdgfa_tmpl_impl_h
#define skyai_tdgfa_tmpl_impl_h
//-------------------------------------------------------------------------------------------
#include <skyai/modules_std/td_generic_fa.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
namespace tdgfa_tmpl_detail
{
using namespace std;
// using namespace boost;


//===========================================================================================
// template <typename t_state, typename t_action, typename t_parameter>
// class MTDGenericFuncApprox
//===========================================================================================

#define TEMPLATE_DEC  template <typename t_action>
#define XMODULE       MTDGenericFuncApprox <t_action>
#define XMODULE_STR  "MTDGenericFuncApprox <t_action>"

TEMPLATE_DEC
override void XMODULE::slot_initialize_exec (void)
{
  // mem_.EpisodeNumber= 0;
  return_in_episode_= 0.0l;
  is_end_of_episode_= false;
  is_active_= false;
  current_action_value_= 0.0l;
}
//-------------------------------------------------------------------------------------------

TEMPLATE_DEC
override void XMODULE::slot_start_episode_exec (void)
{
  return_in_episode_= 0.0l;
  actions_in_episode_= 0;
  current_action_value_= 0.0l;
  reset_episode();

  prepare_next_action();
  is_end_of_episode_= false;
  is_active_= true;
  current_action_value_= nextQ;
  signal_execute_action.ExecAll (select_action());
}
//-------------------------------------------------------------------------------------------

TEMPLATE_DEC
override void XMODULE::slot_finish_episode_exec (void)
{
  if (is_active_)
    is_end_of_episode_= true;
}
//-------------------------------------------------------------------------------------------

TEMPLATE_DEC
override void XMODULE::slot_finish_episode_immediately_exec (void)
{
  if (is_active_)
  {
    is_end_of_episode_= true;
    TParent::slot_finish_action.Exec();
  }
}
//-------------------------------------------------------------------------------------------

TEMPLATE_DEC
override void XMODULE::slot_finish_action_exec (void)
{
  if (is_active_)
  {
    /*! reward */
    TSingleReward reward (0.0l);
    reward= get_reward();

    /*! select next action; since it is used in Sarsa */
    prepare_next_action ();

    if (!is_end_of_episode_)
    {
      current_action_value_= nextQ;
      signal_execute_action.ExecAll (next_action);
    }

    update (reward);
    select_action();
    ++actions_in_episode_;

    signal_end_of_action.ExecAll();

    if (is_end_of_episode_)
    {
      ++mem_.EpisodeNumber;
      is_active_= false;
      signal_end_of_episode.ExecAll();
    }
  }
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
// MTDGenericFuncApprox implementation of protected member functions
//===========================================================================================

TEMPLATE_DEC inline TReal XMODULE::get_alpha (void) const
{
  TReal alpha (0.0l);
  alpha= conf_.Alpha * exp (-conf_.AlphaDecreasingFactor * static_cast<TReal>(mem_.EpisodeNumber));
  return ApplyRange(alpha,conf_.AlphaMin,conf_.Alpha);
}
//-------------------------------------------------------------------------------------------

TEMPLATE_DEC
TValue XMODULE::update_with_ql (const TSingleReward &R)
{
  //! update Q
  const TReal alpha (get_alpha());

  TValue evt = R + conf_.Gamma * nextV - oldV;
  TValue eqt = R + conf_.Gamma * nextV - oldQ;  // = tderror

  get_avf_zero_parameter (difference_());

  if (conf_.UsingEligibilityTrace)
  {
    difference_().AddProd (alpha * evt, activity_trace());
    difference_().AddProd (alpha * eqt, old_grad());
    activity_trace() += old_grad();
    activity_trace() *= (conf_.Gamma * conf_.Lambda);
    if (conf_.UsingReplacingTrace)
    {
      get_avf_replacing_trace (activity_trace());
    }
  }
  else
  {
    difference_().AddProd (alpha * eqt, old_grad());
  }

  signal_avf_add_to_parameter.ExecAll (difference_());

  return eqt;
}
//-------------------------------------------------------------------------------------------

TEMPLATE_DEC
TValue XMODULE::update_with_sarsa (const TSingleReward &R)
{
  const TReal alpha (get_alpha());

  // normal Sarsa process
  TValue eqt = R + conf_.Gamma * nextQ - oldQ;  // = tderror

  get_avf_zero_parameter (difference_());

  if (conf_.UsingEligibilityTrace)
  {
    activity_trace() *= (conf_.Gamma * conf_.Lambda);
    activity_trace() += old_grad();
    if (conf_.UsingReplacingTrace)
    {
      get_avf_replacing_trace (activity_trace());
    }
    difference_().AddProd (alpha * eqt, activity_trace());
  }
  else
  {
    difference_().AddProd (alpha * eqt, old_grad());
  }

  signal_avf_add_to_parameter.ExecAll (difference_());

  return eqt;
}
//-------------------------------------------------------------------------------------------

//! select \p TAgentTDGFA::next_action from real actions and internal actions
TEMPLATE_DEC
/*virtual*/ void XMODULE::prepare_next_action ()
{
  // next_state= get_state();

  //! calculate action value at current state x
  /* note: the gradient (of the action value function) used to update should be calculated for
      previous (state,action) pair.  Thus, it should be stored here. */
  get_avf_select_action (&next_action, TStateActionAttribute(&nextQ, &nextV, &(next_grad())));

  TReal grad_norm= next_grad().Norm();
  if (conf_.GradientMax > 0.0l && grad_norm > conf_.GradientMax)  next_grad() *= conf_.GradientMax/grad_norm;
}
//-------------------------------------------------------------------------------------------

//! execute this function to start new trial
TEMPLATE_DEC
/*virtual*/ void XMODULE::reset_episode ()
{
  difference_.Allocate();

  if (conf_.UsingEligibilityTrace)
    activity_trace.Allocate();

  action_sequence_index= 0;
}
//-------------------------------------------------------------------------------------------

TEMPLATE_DEC
/*virtual*/ typename XMODULE::TAction XMODULE::select_action ()
{
  // start new state (\in func_approx) transition
  old_action= next_action;  // next_action has been selected in prepare_next_action(x)
  oldQ= nextQ;
  oldV= nextV;
  old_grad= next_grad;

  return old_action;
}
//-------------------------------------------------------------------------------------------

TEMPLATE_DEC
/*virtual*/ void XMODULE::update (const TSingleReward &reward)
{

  return_in_episode_+= reward;

  /* update each module with TD */
  if (is_updatable())
  {
    switch(conf_.LearningAlgorithm)
    {
      case laQLearning      : td_error_= update_with_ql(reward); break;
      case laSarsa          : td_error_= update_with_sarsa(reward); break;
      default :
        LERROR("fatal in " XMODULE_STR "::update: invalid LearningAlgorithm "<<conf_.LearningAlgorithm); lexit(df);
    }
  }

  if (is_end_of_episode_)
  {
    /* policy gradient */
  }
}
//-------------------------------------------------------------------------------------------


#undef XMODULE_STR
#undef XMODULE
#undef TEMPLATE_DEC
//-------------------------------------------------------------------------------------------
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MTDGenericFuncApprox,TContinuousAction)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MTDGenericFuncApprox,TDiscreteAction)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MTDGenericFuncApprox,TCompositeAction)

SKYAI_ADD_MODULE(MTDGenericFuncApprox_TContinuousAction)
SKYAI_ADD_MODULE(MTDGenericFuncApprox_TDiscreteAction)
SKYAI_ADD_MODULE(MTDGenericFuncApprox_TCompositeAction)
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // end of namespace tdgfa_tmpl_detail
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_tdgfa_tmpl_impl_h
//-------------------------------------------------------------------------------------------
