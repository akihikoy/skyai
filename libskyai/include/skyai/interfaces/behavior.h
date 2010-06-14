//-------------------------------------------------------------------------------------------
/*! \file    behavior.h
    \brief   libskyai - interfaces for behavior modules
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Oct.09, 2009-

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
#ifndef skyai_behavior_h
#define skyai_behavior_h
//-------------------------------------------------------------------------------------------
#include <skyai/skyai.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------


//===========================================================================================
//!\brief Template interface for a behavior module
template <typename t_action>
class MBehaviorInterface : public TModuleInterface
//===========================================================================================
{
public:
  // typedef t_state                    TState;
  typedef t_action                      TAction;
  typedef TModuleInterface              TParent;
  typedef MBehaviorInterface <TAction>  TThis;
  SKYAI_MODULE_NAMES(MBehaviorInterface)

  MBehaviorInterface (const std::string &v_instance_name)
    : TParent                          (v_instance_name),
      slot_initialize                  (*this),
      slot_start_episode               (*this),
      slot_finish_episode              (*this),
      slot_finish_episode_immediately  (*this),
      slot_finish_action               (*this),
      signal_end_of_episode            (*this),
      signal_execute_action            (*this),
      signal_end_of_action             (*this),
      // in_state                         (*this),
      in_updatable                     (*this)
    {
      add_slot_port (slot_initialize);
      add_slot_port (slot_start_episode);
      add_slot_port (slot_finish_episode);
      add_slot_port (slot_finish_episode_immediately);
      add_slot_port (slot_finish_action);

      add_signal_port (signal_end_of_episode);
      add_signal_port (signal_execute_action);
      add_signal_port (signal_end_of_action);

      // add_in_port (in_state);
      add_in_port (in_updatable);
    }

  virtual ~MBehaviorInterface() {}

protected:

  //!\brief if this slot catch a signal, this module is initialized
  MAKE_SLOT_PORT(slot_initialize, void, (void), (), TThis);

  //!\brief if this slot catch a signal, an episode is started
  MAKE_SLOT_PORT(slot_start_episode, void, (void), (), TThis);

  //!\brief if this slot catch a signal, the current episode is finished after the current action is finished
  MAKE_SLOT_PORT(slot_finish_episode, void, (void), (), TThis);

  //!\brief if this slot catch a signal, the current episode is finished immediately
  MAKE_SLOT_PORT(slot_finish_episode_immediately, void, (void), (), TThis);


  //!\brief if this slot catch a signal, the current action is finished
  MAKE_SLOT_PORT(slot_finish_action, void, (void), (), TThis);



  //!\brief this signal is emitted just after the episode is finished
  MAKE_SIGNAL_PORT(signal_end_of_episode, void(void), TThis);

  //!\brief this signal is emitted to request to execute the specified action
  MAKE_SIGNAL_PORT(signal_execute_action, void(const TAction&), TThis);

  /*!\brief this signal is emitted at the end of each action
      \note signal_end_of_action should be emitted before signal_end_of_episode */
  MAKE_SIGNAL_PORT(signal_end_of_action, void(void), TThis);


  //!\brief input whether this module is updatable or not by this port
  MAKE_IN_PORT(in_updatable, const TBool& (void), TThis);


  virtual void slot_initialize_exec (void) = 0;
  virtual void slot_start_episode_exec (void) = 0;
  virtual void slot_finish_episode_exec (void) = 0;
  virtual void slot_finish_episode_immediately_exec (void) = 0;
  virtual void slot_finish_action_exec (void) = 0;

};
//-------------------------------------------------------------------------------------------



//===========================================================================================
//!\brief Template interface for a reinforcement learning module
template <typename t_action>
class MReinforcementLearningInterface
    : public MBehaviorInterface<t_action>
//===========================================================================================
{
public:
  typedef MBehaviorInterface<t_action>                TParent;
  typedef MReinforcementLearningInterface <t_action>  TThis;
  SKYAI_MODULE_NAMES(MReinforcementLearningInterface)

  MReinforcementLearningInterface (const std::string &v_instance_name)
    : TParent                       (v_instance_name),
      in_reward                     (*this)
    {
      add_in_port (in_reward);
    }

  virtual ~MReinforcementLearningInterface() {}

protected:

  //!\brief input a reward by this port
  MAKE_IN_PORT(in_reward, const TSingleReward& (void), TThis);
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
//!\brief Interface for a reinforcement learning module (discrete action)
class MRLInterface_TDiscreteAction
    : public MReinforcementLearningInterface<TDiscreteAction>
//===========================================================================================
{
public:
  typedef MReinforcementLearningInterface <
                                TDiscreteAction>  TParent;
  typedef MRLInterface_TDiscreteAction  TThis;
  SKYAI_MODULE_NAMES(MRLInterface_TDiscreteAction)

  MRLInterface_TDiscreteAction (const std::string &v_instance_name)
    : TParent                       (v_instance_name),
      in_action_set_size            (*this)
    {
      add_in_port (in_action_set_size);
    }

  virtual ~MRLInterface_TDiscreteAction() {}

protected:


  //!\brief input a size of action-set by this port
  MAKE_IN_PORT(in_action_set_size, const TInt& (void), TThis);
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
/*!\brief Interface for a reinforcement learning module with a linear function approximator (discrete action)
    \note the linear function approximator: Q(x,a) = feature(x).transpose() * q(a)
      where feature(x) is the output vector (the feature vector) at the state x which is s-dimensional,
      and q(a) is a parameter vector for the action a which is also s-dimensional. */
class MRLLinearFuncApproxInterface_TDiscreteAction
    : public MRLInterface_TDiscreteAction
//===========================================================================================
{
public:
  typedef TRealVector                                   TFeature; //!< type to represent an output of a function approximator
  typedef MRLInterface_TDiscreteAction                  TParent;
  typedef MRLLinearFuncApproxInterface_TDiscreteAction  TThis;
  SKYAI_MODULE_NAMES(MRLLinearFuncApproxInterface_TDiscreteAction)

  MRLLinearFuncApproxInterface_TDiscreteAction (const std::string &v_instance_name)
    : TParent                       (v_instance_name),
      in_feature                    (*this)
    {
      add_in_port (in_feature);
    }

  virtual ~MRLLinearFuncApproxInterface_TDiscreteAction() {}

protected:

  //!\brief input a feature (an output of a function approximator) by this port
  MAKE_IN_PORT(in_feature, const TFeature& (void), TThis);
};
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_behavior_h
//-------------------------------------------------------------------------------------------
