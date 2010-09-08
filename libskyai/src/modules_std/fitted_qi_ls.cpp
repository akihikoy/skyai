//-------------------------------------------------------------------------------------------
/*! \file    fitted_qi_ls.cpp
    \brief   libskyai - Fitted Q Iteration implementation with a gradient descent method for the least square error
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Apr.13, 2010-

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
#include <skyai/modules_std/fitted_qi_ls.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
using namespace std;
// using namespace boost;
namespace fitted_qi_ls_detail
{




//===========================================================================================
// template <typename t_state, typename t_action, typename t_parameter>
// class MFittedQIterationSL
//===========================================================================================

#define TEMPLATE_DEC  template <typename t_state, typename t_action>
#define XMODULE       MFittedQIterationSL <t_state, t_action>
#define XMODULE_STR  "MFittedQIterationSL <t_state, t_action>"

TEMPLATE_DEC
override void XMODULE::slot_initialize_exec (void)
{
  // mem_.EpisodeNumber= 0;
  // return_in_episode_= 0.0l;
  reward_statistics_in_episode_.Clear();
  max_reward_deviation_= 0.0;
  is_end_of_episode_= false;
  is_active_= false;
  current_action_value_= 0.0l;
  is_puppet_= false;
}
//-------------------------------------------------------------------------------------------

TEMPLATE_DEC
override void XMODULE::slot_start_episode_exec (void)
{
  // return_in_episode_= 0.0l;
  reward_statistics_in_episode_.Clear();
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
    if (is_updatable())  reward= get_reward();

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
// MFittedQIterationSL implementation of protected member functions
//===========================================================================================

TEMPLATE_DEC inline TReal XMODULE::get_alpha (void) const
{
  TReal alpha (0.0l);
  alpha= conf_.Alpha * exp (-conf_.AlphaDecreasingFactor * static_cast<TReal>(mem_.EpisodeNumber));
  return ApplyRange(alpha,conf_.AlphaMin,conf_.Alpha);
}
//-------------------------------------------------------------------------------------------


//! select \p TAgentTDGFA::next_action from real actions and internal actions
TEMPLATE_DEC
/*virtual*/ void XMODULE::prepare_next_action ()
{
  next_state= get_state();

  //! calculate action value at current state x
  if (slot_puppet_action.ConnectionSize()==0)
  {
    TStateActionAttribute next_attrib;
    next_attrib.ActionValue= &nextQ;
    get_avf_select_action (&next_action, next_attrib);
  }
  else
  {
    LASSERT(is_puppet_);
    next_action= puppet_action_;
  }
}
//-------------------------------------------------------------------------------------------

//! execute this function to start new trial
TEMPLATE_DEC
/*virtual*/ void XMODULE::reset_episode ()
{
  difference_.Allocate();
  gradient_.Allocate();

  current_sample_.clear();
}
//-------------------------------------------------------------------------------------------

TEMPLATE_DEC
/*virtual*/ typename XMODULE::TAction XMODULE::select_action ()
{
  old_state= next_state;
  old_action= next_action;  // next_action has been selected in prepare_next_action()
  oldQ= nextQ;

  return old_action;
}
//-------------------------------------------------------------------------------------------

TEMPLATE_DEC
/*virtual*/ void XMODULE::update (const TSingleReward &reward)
{
  // return_in_episode_+= reward;
  reward_statistics_in_episode_.Add(reward);

  if (is_updatable())
  {
    // update data (push back a sample into fqi_data_)
    if (current_sample_.empty())  // only the first step in an episode
      current_sample_.push_back (TFQISample<TState,TAction>(0.0l,old_state,old_action));
    current_sample_.push_back (TFQISample<TState,TAction>(reward,next_state,next_action));
  }

  if (is_end_of_episode_)
  {
    if (is_updatable())
    {
      manipulate_samples();

      if ((mem_.EpisodeNumber+1)%conf_.FQICycle==0)
      {
        // apply fitted Q-iteration with fqi_data_ ...
        for (int NQI(0); NQI<conf_.MaxNumberOfQIteration; ++NQI)
        {
          // calculate state values w.r.t. the current action value function
          update_state_values (conf_.NumberOfUsedSamples);
          // supervised learning iteration
          exec_supervised_learning (conf_.NumberOfUsedSamples);
        }  // Q-iteration

      }  // FQICycle
    }  // is_updatable
  }  // is_end_of_episode_
}
//-------------------------------------------------------------------------------------------

TEMPLATE_DEC void XMODULE::manipulate_samples()
{
LDBGVAR(current_sample_.size());
LDBGVAR(fqi_data_.Data().size());
LDEBUG("reward_statistics_in_episode_="<<std::endl<<reward_statistics_in_episode_);
  max_reward_deviation_= std::max(max_reward_deviation_, reward_statistics_in_episode_.StdDeviation());
LDBGVAR(max_reward_deviation_);
  if (current_sample_.size()>1)
  {
    current_sample_.back().IsTerminal= true;
    if (reward_statistics_in_episode_.StdDeviation()>conf_.MinRewardDeviationRate*max_reward_deviation_
          || reward_statistics_in_episode_.NumberOfSamples==1)
    {
      typename TFQIData<TState,TAction>::T::TData::iterator
            new_itr= fqi_data_.Push(reward_statistics_in_episode_, current_sample_);  // add episode into the sample set
      if (new_itr!=fqi_data_.Data().begin())
      {
        TReal diff_mean (new_itr->Key.Mean);
        --new_itr; // results older episode
        diff_mean= real_fabs(diff_mean-new_itr->Key.Mean);
        if (diff_mean < conf_.SameSampleThreshold*max_reward_deviation_)
          fqi_data_.Data().erase(new_itr);
        // if diff_mean is small, older episode is removed from the sample set.
        // note that the episode with the maximum return is not be removed in anytime.
      }
    }
  }
  if (conf_.EliminateSampleByDeviation)
  {
    typename TFQIData<TState,TAction>::T::TData::iterator eps_itr(fqi_data_.Data().begin());
    while (eps_itr!=fqi_data_.Data().end())
    {
      if (eps_itr->Key.StdDeviation() < conf_.MinRewardDeviationRate*max_reward_deviation_)
        eps_itr= fqi_data_.Data().erase(eps_itr);
      else
        ++eps_itr;
    }
  }
  if (conf_.NumberOfUsedSamples > 0)
  {
    while (static_cast<TInt>(fqi_data_.Data().size()) > conf_.NumberOfUsedSamples)
      fqi_data_.Data().pop_front();
  }
}
//-------------------------------------------------------------------------------------------

//! calculate state values w.r.t. the current action value function for each sample particle
TEMPLATE_DEC void XMODULE::update_state_values (int number_of_used_samples)
{
  for (typename TFQIData<TState,TAction>::T::TData::iterator eps_itr(list_itr_at(fqi_data_.Data(),-number_of_used_samples)); eps_itr!=fqi_data_.Data().end(); ++eps_itr)
  {
    typename TFQIData<TState,TAction>::T::TList::iterator  old_itr(eps_itr->List.begin());
    typename TFQIData<TState,TAction>::T::TList::iterator  next_itr(eps_itr->List.begin());
    ++next_itr;
    TValue state_value;
    TStateActionAttribute next_attrib;
    next_attrib.StateValue= &state_value;
    for (; next_itr!=eps_itr->List.end(); ++next_itr,++old_itr)
    {
      get_avf_greedy (next_itr->State, NULL, next_attrib);
      old_itr->StateValue= next_itr->Reward + conf_.Gamma * state_value;
    }
  }
}
//-------------------------------------------------------------------------------------------

//! supervised learning iteration
TEMPLATE_DEC void XMODULE::exec_supervised_learning (int number_of_used_samples)
{
  const TReal alpha (get_alpha());

  if (conf_.LSMethodType == mtBatch)
  {
    for (int NSL(0); NSL<conf_.MaxNumberOfSLIteration; ++NSL)
    {
      get_avf_zero_parameter (difference_());
      get_avf_zero_parameter (gradient_());
      TValue sum_tderror(0.0l), tderror;
      TValue action_value;
      TStateActionAttribute attrib;
      attrib.ActionValue= &action_value;
      attrib.Gradient= &(gradient_());
      for (typename TFQIData<TState,TAction>::T::i_iterator itr(fqi_data_.ibegin(-number_of_used_samples)); itr!=fqi_data_.iend(); ++itr)
      {
        if (itr->IsTerminal)  continue;
        get_avf_evaluate (itr->State, itr->Action, attrib);
        tderror= itr->StateValue - action_value;
        sum_tderror+= real_fabs(tderror);
        difference_().AddProd (tderror, gradient_());
      }
      TReal sqerr_grad_norm(difference_().Norm());
      if (conf_.SqErrGradientNormLimit > 0.0l && sqerr_grad_norm > conf_.SqErrGradientNormLimit)
        difference_()*= alpha * conf_.SqErrGradientNormLimit/sqerr_grad_norm;
      else
        difference_()*= alpha;
      signal_avf_add_to_parameter.ExecAll (difference_());
LDBGVAR(sqerr_grad_norm);
LDBGVAR(sum_tderror);
    }  // SL-iteration
  }  // conf_.LSMethodType == mtBatch
  else if (conf_.LSMethodType == mtPointByPoint)
  {
    for (int NSL(0); NSL<conf_.MaxNumberOfSLIteration; ++NSL)
    {
      get_avf_zero_parameter (difference_());
      get_avf_zero_parameter (gradient_());
      TValue sum_tderror(0.0l), tderror;
      TValue action_value;
      TStateActionAttribute attrib;
      attrib.ActionValue= &action_value;
      attrib.Gradient= &(gradient_());
      for (typename TFQIData<TState,TAction>::T::i_iterator itr(fqi_data_.ibegin(-number_of_used_samples)); itr!=fqi_data_.iend(); ++itr)
      {
        if (itr->IsTerminal)  continue;
        get_avf_evaluate (itr->State, itr->Action, attrib);
        tderror= itr->StateValue - action_value;
        sum_tderror+= real_fabs(tderror);
        difference_()= gradient_();

        TReal sqerr_grad_norm(tderror * gradient_().Norm());
        if (conf_.SqErrGradientNormLimit > 0.0l && sqerr_grad_norm > conf_.SqErrGradientNormLimit)
          difference_()*= alpha* conf_.SqErrGradientNormLimit/sqerr_grad_norm;
        else
          difference_()*= alpha*tderror;
        signal_avf_add_to_parameter.ExecAll (difference_());
      }
LDBGVAR(sum_tderror);
    }  // SL-iteration
  }  // conf_.LSMethodType == mtPointByPoint
}
//-------------------------------------------------------------------------------------------


#undef XMODULE_STR
#undef XMODULE
#undef TEMPLATE_DEC
//-------------------------------------------------------------------------------------------
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MFittedQIterationSL,TContinuousState,TContinuousAction)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MFittedQIterationSL,TContinuousState,TDiscreteAction)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MFittedQIterationSL,TCompositeState,TCompositeAction)

SKYAI_ADD_MODULE(MFittedQIterationSL_TContinuousState_TContinuousAction)
SKYAI_ADD_MODULE(MFittedQIterationSL_TContinuousState_TDiscreteAction)
SKYAI_ADD_MODULE(MFittedQIterationSL_TCompositeState_TCompositeAction)
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
}  // end of fitted_qi_ls_detail
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

