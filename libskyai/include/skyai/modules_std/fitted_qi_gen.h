//-------------------------------------------------------------------------------------------
/*! \file    fitted_qi_gen.h
    \brief   libskyai - Fitted Q Iteration implementation for a generic function approximator (header)
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
#ifndef skyai_fitted_qi_gen_h
#define skyai_fitted_qi_gen_h
//-------------------------------------------------------------------------------------------
#include <skyai/interfaces/action_value_func.h>
#include <skyai/interfaces/behavior.h>
#include <skyai/types.h>
#include <lora/small_classes.h>
#include <lora/variable_space_impl.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
namespace fitted_qi_detail
{
//-------------------------------------------------------------------------------------------

template <typename t_state, typename t_action>
struct TFQISample
{
  TSingleReward   Reward;
  t_state         State;
  t_action        Action;
  TValue          StateValue;  // target state value at the State
  bool            IsTerminal;  // true if the terminal sample of the episode
  TFQISample (const TSingleReward v_reward, const t_state &v_state, const t_action &v_action)
    : Reward(v_reward), State(v_state), Action(v_action), StateValue(0.0l), IsTerminal(false)  {}
};

class TFQIRewardStatistics : public TStatisticsFilter<TSingleReward>
{
public:
  bool operator< (const TFQIRewardStatistics &rhs) const {return Sum()<rhs.Sum();}
  bool operator> (const TFQIRewardStatistics &rhs) const {return Sum()>rhs.Sum();}
};

template <typename t_state, typename t_action>
struct TFQIData
{
  // typedef TListPQueue<TSingleReward,TFQISample<t_state, t_action> >  T;
  typedef TListPQueue<TFQIRewardStatistics,TFQISample<t_state, t_action> >  T;
};
//-------------------------------------------------------------------------------------------


//===========================================================================================
//!\brief Configurations of MFittedQIterationGen
class TFittedQIterationGenConfigurations
//===========================================================================================
{
public:

  TReal                 Gamma;

  TInt                  FQICycle;
  TInt                  MaxNumberOfQIteration;

  TReal                 MinRewardDeviationRate;  //!< if the std-deviation of rewards in an episode is less than (MinRewardDeviationRate x max std-deviation), the episode is not used as a sample
  TReal                 SameSampleThreshold;  //!< if the difference of mean reward between two episodes is less than (SameSampleThreshold x max std-deviation), these episodes assumed to be the same, and older one is removed
  bool                  EliminateSampleByDeviation;  //!< if true, samples are eliminated by the same manner as MinRewardDeviationRate
  TInt                  NumberOfUsedSamples;  //!< if not zero, NumberOfUsedSamples biggest-return samples are used to calculate

  TFittedQIterationGenConfigurations (var_space::TVariableMap &mmap)
    :
      Gamma                         (0.9l),
      FQICycle                      (1),
      MaxNumberOfQIteration         (1),
      MinRewardDeviationRate        (0.2l),
      SameSampleThreshold           (0.0l),
      EliminateSampleByDeviation    (true),
      NumberOfUsedSamples           (0)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( Gamma                           );

      ADD( FQICycle                        );
      ADD( MaxNumberOfQIteration           );
      ADD( MinRewardDeviationRate          );
      ADD( SameSampleThreshold             );
      ADD( EliminateSampleByDeviation      );
      ADD( NumberOfUsedSamples             );
      #undef ADD
    }

};
//-------------------------------------------------------------------------------------------

//===========================================================================================
//!\brief Memory of MFittedQIterationGen
class TFittedQIterationGenMemory
//===========================================================================================
{
public:

  TInt  EpisodeNumber;  //!< episode number

  TFittedQIterationGenMemory (var_space::TVariableMap &mmap)
    :
      EpisodeNumber (0)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( EpisodeNumber );
      #undef ADD
    }

};
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief template reinforcement learning module using fitted Q iteration with a gradient descent method for the least square error
    \note the function approximator is NOT included in this module (use MAVFLinearDiscAction, etc.)
    \todo TODO: implement out_td_error_get function
*/
template <typename t_state, typename t_action>
class MFittedQIterationGen
    : public MReinforcementLearningInterface <t_action>
//===========================================================================================
{
public:
  typedef MReinforcementLearningInterface <
                                      t_action>  TParent;
  typedef t_state                                TState;
  typedef typename TParent::TAction              TAction;
  typedef MFittedQIterationGen                   TThis;
  SKYAI_MODULE_NAMES(MFittedQIterationGen)

  MFittedQIterationGen (const std::string &v_instance_name)
    : TParent                       (v_instance_name),
      conf_                         (TParent::param_box_config_map()),
      mem_                          (TParent::param_box_memory_map()),
      in_state                      (*this),
      slot_puppet_action            (*this),
      signal_train_with_data        (*this),
      in_avf_evaluate               (*this),
      in_avf_greedy                 (*this),
      in_avf_select_action          (*this),
      out_episode_number            (*this),
      out_return_in_episode         (*this),
      out_td_error                  (*this),
      out_current_action_value      (*this),
      fqi_data_                     (TFQIData<TState,TAction>::T::soAscending)
    {
      this->add_in_port (in_state);
      this->add_slot_port (slot_puppet_action);

      this->add_signal_port (signal_train_with_data);
      this->add_in_port (in_avf_evaluate         );
      this->add_in_port (in_avf_greedy           );
      this->add_in_port (in_avf_select_action    );

      this->add_out_port (out_episode_number);
      this->add_out_port (out_return_in_episode);
      this->add_out_port (out_td_error);
      this->add_out_port (out_current_action_value);
    }

protected:

  TFittedQIterationGenConfigurations  conf_;
  TFittedQIterationGenMemory          mem_;

  //!\brief state input (must be the same as one which linked to the action value function module)
  MAKE_IN_PORT(in_state, const TState& (void), TThis);

  //!\brief if connected, this module behaves as if taking the action specified by this port
  MAKE_SLOT_PORT(slot_puppet_action, void, (const TAction &a), (a), TThis);

  // [-- ports for a function approximator
  /*!\brief update the parameter from the training sample data. */
  MAKE_SIGNAL_PORT(signal_train_with_data, void (const std::list<TActionValueFuncTrainingSample<TState,TAction> > &data), TThis);

  MAKE_IN_PORT(in_avf_evaluate, void (const TState &x, const TAction &a, TStateActionAttribute attrib), TThis);

  //!\brief calculate a greedy action at x, assign it into *greedy (if not NULL)
  MAKE_IN_PORT(in_avf_greedy, void (const TState &x, TAction *greedy, TStateActionAttribute attrib), TThis);

  /*!\brief select an action at the current state by the current polocy
      \param [out]a  : if not NULL, selected action is stored
      \param [out]attrib  : the action value, the state value, and the gradient are stored */
  MAKE_IN_PORT(in_avf_select_action, void (TAction *a, TStateActionAttribute attrib), TThis);
  // ports for a function approximator --]

  MAKE_OUT_PORT(out_episode_number, const TInt&, (void), (), TThis);
  MAKE_OUT_PORT(out_return_in_episode, const TSingleReward&, (void), (), TThis);
  MAKE_OUT_PORT(out_td_error, const TValue&, (void), (), TThis);
  MAKE_OUT_PORT(out_current_action_value, const TValue&, (void), (), TThis);

  override void slot_initialize_exec (void);
  override void slot_start_episode_exec (void);
  override void slot_finish_episode_exec (void);
  override void slot_finish_episode_immediately_exec (void);
  override void slot_finish_action_exec (void);
  void slot_puppet_action_exec (const TAction &a)  {puppet_action_= a; is_puppet_= true;}
  const TInt& out_episode_number_get (void) const {return mem_.EpisodeNumber;}
  const TSingleReward& out_return_in_episode_get (void) const {return reward_statistics_in_episode_.Sum();}  // return_in_episode_
  const TValue& out_td_error_get (void) const {return td_error_;}
  const TValue& out_current_action_value_get (void) const {return current_action_value_;}

  using TParent::signal_end_of_episode   ;
  using TParent::signal_execute_action   ;
  using TParent::signal_end_of_action    ;
  using TParent::in_updatable            ;

  using TParent::in_reward               ;

  using TParent::ModuleUniqueCode        ;

  // generate get_##x_in (e.g. get_reward)
  #define GET_FROM_IN_PORT(x_in,x_return_type,x_arg_list,x_param_list)                          \
    x_return_type  get_##x_in x_arg_list const                                                  \
      {                                                                                         \
        if (in_##x_in.ConnectionSize()==0)                                                      \
          {LERROR("in "<<ModuleUniqueCode()<<", in_" #x_in " must be connected."); lexit(df);}  \
        return in_##x_in.GetFirst x_param_list;                                                 \
      }

  // GET_FROM_IN_PORT(state, const TState&, (void), ())

  GET_FROM_IN_PORT(reward, const TSingleReward&, (void), ())

  GET_FROM_IN_PORT(state, const TState&, (void), ())

  GET_FROM_IN_PORT(avf_evaluate, void, (const TState &x, const TAction &a, TStateActionAttribute attrib), (x, a, attrib))
  GET_FROM_IN_PORT(avf_greedy, void, (const TState &x, TAction *greedy, TStateActionAttribute attrib), (x, greedy, attrib))
  GET_FROM_IN_PORT(avf_select_action, void, (TAction *a, TStateActionAttribute attrib), (a,attrib))

  #undef GET_FROM_IN_PORT

  virtual bool is_updatable() const
    {
      if (in_updatable.ConnectionSize()==0)  return true;
      return in_updatable.GetFirst();
    }


  TState                old_state;
  TState                next_state;
  TAction               old_action;
  TAction               next_action;
  TValue                oldQ;
  TValue                nextQ;

  bool                  is_puppet_;
  TAction               puppet_action_;

  typename TFQIData<TState,TAction>::T         fqi_data_;
  typename TFQIData<TState,TAction>::T::TList  current_sample_;

  bool                  is_active_;
  // TSingleReward         return_in_episode_;
  TFQIRewardStatistics  reward_statistics_in_episode_;
  TReal                 max_reward_deviation_;
  TInt                  actions_in_episode_;
  bool                  is_end_of_episode_;
  TValue                td_error_;
  TValue                current_action_value_;

  //! select \p TAgentTDGFA::next_action from real actions and internal actions
  virtual void     prepare_next_action ();

  // execution methods
  virtual void     reset_episode (); //!< execute this function to start new trial
  virtual TAction  select_action ();
  virtual void     update (const TSingleReward &reward);

  // sub-execution methods
  //! add current_sample_ and eliminate some samples
  virtual void manipulate_samples ();
  //! calculate state values w.r.t. the current action value function for each sample particle
  virtual void update_state_values (int number_of_used_samples);
  //! supervised learning iteration
  virtual void exec_supervised_learning (int number_of_used_samples);

};
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MFittedQIterationGen,TContinuousState,TContinuousAction)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MFittedQIterationGen,TContinuousState,TDiscreteAction)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MFittedQIterationGen,TCompositeState,TCompositeAction)
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
}  // end of fitted_qi_detail
//-------------------------------------------------------------------------------------------
}  // end of loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_fitted_qi_gen_h
//-------------------------------------------------------------------------------------------
