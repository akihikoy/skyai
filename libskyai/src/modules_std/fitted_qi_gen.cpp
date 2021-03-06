//-------------------------------------------------------------------------------------------
/*! \file    fitted_qi_gen.cpp
    \brief   libskyai - Fitted Q Iteration implementation for a generic function approximator (source)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Oct.11, 2013

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
#include <skyai/modules_std/fitted_qi_gen.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
using namespace std;
// using namespace boost;
namespace fitted_qi_detail
{


//===========================================================================================
// template <typename t_state, typename t_action>
// class MFittedQIterationGen
//===========================================================================================

#define TEMPLATE_DEC  template <typename t_state, typename t_action>
#define XMODULE       MFittedQIterationGen <t_state, t_action>
#define XMODULE_STR  "MFittedQIterationGen <t_state, t_action>"

TEMPLATE_DEC
override void XMODULE::slot_initialize_exec (void)
{
  // mem_.EpisodeNumber= 0;
  // return_in_episode_= 0.0l;
  reward_statistics_in_episode_.Reset();
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
  reward_statistics_in_episode_.Reset();
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
// MFittedQIterationGen implementation of protected member functions
//===========================================================================================

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
  reward_statistics_in_episode_.Step(reward);

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

TEMPLATE_DEC
/*virtual*/ void XMODULE::manipulate_samples()
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
          || reward_statistics_in_episode_.NumSamples()==1)
    {
      typename TFQIData<TState,TAction>::T::TData::iterator
            new_itr= fqi_data_.Push(reward_statistics_in_episode_, current_sample_);  // add episode into the sample set
      if (new_itr!=fqi_data_.Data().begin())
      {
        TReal diff_mean (new_itr->Key.Mean());
        --new_itr; // results older episode
        diff_mean= real_fabs(diff_mean-new_itr->Key.Mean());
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
TEMPLATE_DEC
/*virtual*/ void XMODULE::update_state_values (int number_of_used_samples)
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
TEMPLATE_DEC
/*virtual*/ void XMODULE::exec_supervised_learning (int number_of_used_samples)
{
  typedef TActionValueFuncTrainingSample<TState,TAction> TS;
  std::list<TS> data;

  for (typename TFQIData<TState,TAction>::T::i_iterator itr(fqi_data_.ibegin(-number_of_used_samples)); itr!=fqi_data_.iend(); ++itr)
  {
    if (itr->IsTerminal)  continue;
    data.push_back(TS(itr->State, itr->Action, itr->StateValue));
  }

  signal_train_with_data.ExecAll(data);
}
//-------------------------------------------------------------------------------------------


#undef XMODULE_STR
#undef XMODULE
#undef TEMPLATE_DEC
//-------------------------------------------------------------------------------------------
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MFittedQIterationGen,TContinuousState,TContinuousAction)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MFittedQIterationGen,TContinuousState,TDiscreteAction)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MFittedQIterationGen,TCompositeState,TCompositeAction)

SKYAI_ADD_MODULE(MFittedQIterationGen_TContinuousState_TContinuousAction)
SKYAI_ADD_MODULE(MFittedQIterationGen_TContinuousState_TDiscreteAction)
SKYAI_ADD_MODULE(MFittedQIterationGen_TCompositeState_TCompositeAction)
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of fitted_qi_detail
}  // end of loco_rabbits
//-------------------------------------------------------------------------------------------
