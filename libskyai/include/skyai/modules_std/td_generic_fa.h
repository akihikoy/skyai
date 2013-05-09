//-------------------------------------------------------------------------------------------
/*! \file    tdgfa_tmpl.h
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
*/
//-------------------------------------------------------------------------------------------
#ifndef skyai_td_generic_fa_h
#define skyai_td_generic_fa_h
//-------------------------------------------------------------------------------------------
#include <skyai/interfaces/action_value_func.h>
#include <skyai/interfaces/behavior.h>
#include <skyai/types.h>
#include <lora/variable_space_impl.h>
#include <boost/bind.hpp>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

namespace tdgfa_tmpl_detail
{
  enum TLearningAlgorithm  //! leaning algorithms
    {
      laQLearning=0,   //!< Q-learning
      laSarsa          //!< Sarsa
    };
} // end of namespace tdgfa_tmpl_detail
ENUM_STR_MAP_BEGIN_NS(tdgfa_tmpl_detail, TLearningAlgorithm)
  ENUM_STR_MAP_ADD_NS(tdgfa_tmpl_detail, laQLearning    )
  ENUM_STR_MAP_ADD_NS(tdgfa_tmpl_detail, laSarsa        )
ENUM_STR_MAP_END_NS  (tdgfa_tmpl_detail, TLearningAlgorithm)
SPECIALIZE_TVARIABLE_TO_ENUM(tdgfa_tmpl_detail::TLearningAlgorithm)


//-------------------------------------------------------------------------------------------
namespace tdgfa_tmpl_detail
{
//-------------------------------------------------------------------------------------------


//===========================================================================================
//!\brief Configurations of MTDGenericFuncApprox
template <typename t_action>
class TConfigurations
//===========================================================================================
{
public:

  typedef t_action   TAction;

  TReal                 Gamma;
  TReal                 Alpha;     //!< step-size parameter
  TReal                 AlphaMin; //!< lower bound of step-size parameter
  TReal                 Lambda;    //!< activity trace Lambda (valid when UsingEligibilityTrace=true)
  TReal                 AlphaDecreasingFactor;   //!< damping coefficient of the step-size parameter. larger is decreasing faster (becoming greedy). do not set a value greater than 1.

  TLearningAlgorithm    LearningAlgorithm;
  bool                  UsingEligibilityTrace;  //!< true: using eligibility trace
  bool                  UsingReplacingTrace;   //!< true: using replacing eligibility trace

  TReal                 GradientMax;  //!< Upper bound of a gradient of the action value function

  TConfigurations (var_space::TVariableMap &mmap)
    :
      Gamma                   (0.9l),
      Alpha                   (0.5l),
      AlphaMin                (DBL_TINY),
      Lambda                  (0.01l),
      AlphaDecreasingFactor   (0.002l),
      LearningAlgorithm       (laQLearning),
      UsingEligibilityTrace   (false),
      UsingReplacingTrace     (false),
      GradientMax             (100.0l)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( Gamma                    );
      ADD( Alpha                    );
      ADD( AlphaMin                 );
      ADD( Lambda                   );
      ADD( AlphaDecreasingFactor    );

      ADD( LearningAlgorithm        );
      ADD( UsingEligibilityTrace    );
      ADD( UsingReplacingTrace      );
      ADD( GradientMax              );
      #undef ADD
    }

};
//-------------------------------------------------------------------------------------------

//===========================================================================================
//!\brief Memory of MTDGenericFuncApprox
class TMemory
//===========================================================================================
{
public:

  TInt  EpisodeNumber;  //!< episode number

  TMemory (var_space::TVariableMap &mmap)
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
/*!\brief template reinforcement learning module using temporal difference methods with a generic function approximator
    \note the function approximator is NOT included in this module (use MAVFLinearDiscAction, etc.)
*/
template <typename t_action>
class MTDGenericFuncApprox
    : public MReinforcementLearningInterface <t_action>
//===========================================================================================
{
public:
  typedef MReinforcementLearningInterface <
                                      t_action>  TParent;
  // typedef typename TParent::TState               TState;
  typedef typename TParent::TAction              TAction;
  typedef TActionValueFuncParamInterface         TParameter;
  typedef TActionValueFuncParamMemory            TParameterMemory;
  typedef MTDGenericFuncApprox                   TThis;
  SKYAI_MODULE_NAMES(MTDGenericFuncApprox)

  MTDGenericFuncApprox (const std::string &v_instance_name)
    : TParent                       (v_instance_name),
      conf_                         (TParent::param_box_config_map()),
      mem_                          (TParent::param_box_memory_map()),
      signal_avf_add_to_parameter   (*this),
      // in_avf_parameter_ref          (*this),
      // in_avf_parameter_val          (*this),
      // in_avf_evaluate               (*this),
      // in_avf_greedy                 (*this),
      in_avf_select_action          (*this),
      in_avf_replacing_trace        (*this),
      in_avf_create_parameter       (*this),
      in_avf_zero_parameter         (*this),
      out_episode_number            (*this),
      out_return_in_episode         (*this),
      out_td_error                  (*this),
      out_current_action_value      (*this)
    {
      this->add_signal_port (signal_avf_add_to_parameter);

      // this->add_in_port (in_avf_parameter_ref    );
      // this->add_in_port (in_avf_parameter_val    );
      // this->add_in_port (in_avf_evaluate         );
      // this->add_in_port (in_avf_greedy           );
      this->add_in_port (in_avf_select_action    );
      this->add_in_port (in_avf_replacing_trace  );
      this->add_in_port (in_avf_create_parameter );
      this->add_in_port (in_avf_zero_parameter   );

      this->add_out_port (out_episode_number);
      this->add_out_port (out_return_in_episode);
      this->add_out_port (out_td_error);
      this->add_out_port (out_current_action_value);

      old_grad       .SetAllocator (boost::bind(&TThis::get_avf_create_parameter,this));
      next_grad      .SetAllocator (boost::bind(&TThis::get_avf_create_parameter,this));
      difference_    .SetAllocator (boost::bind(&TThis::get_avf_create_parameter,this));
      activity_trace .SetAllocator (boost::bind(&TThis::get_avf_create_parameter,this));
    }

protected:

  TConfigurations<TAction>  conf_;
  TMemory                   mem_;

  // [-- ports for a function approximator
  MAKE_SIGNAL_PORT(signal_avf_add_to_parameter, void (const TParameter &), TThis);

  // //!\brief output a reference to the parameter
  // MAKE_IN_PORT(in_avf_parameter_ref, const TParameter& (void), TThis);

  // //!\brief assign the parameter to outerparam
  // MAKE_IN_PORT(in_avf_parameter_val, void (TParameter &outerparam), TThis);

  // MAKE_IN_PORT(in_avf_evaluate, void (const TState &x, const TAction &a, TStateActionAttribute attrib), TThis);

  // //!\brief calculate a greedy action at x, assign it into *greedy (if not NULL), assign its action value into q (if not NULL)
  // MAKE_IN_PORT(in_avf_greedy, void (const TState &x, TAction *greedy, TStateActionAttribute attrib), TThis);

  /*!\brief select an action at the current state by the current polocy
      \param [out]a  : if not NULL, selected action is stored
      \param [out]attrib  : the action value, the state value, and the gradient are stored */
  MAKE_IN_PORT(in_avf_select_action, void (TAction *a, TStateActionAttribute attrib), TThis);

  //!\brief apply a replacing trace to an eligibility trace eligibility_trace
  MAKE_IN_PORT(in_avf_replacing_trace, void (TParameter &eligibility_trace), TThis);

  //!\brief create (allocate) a parameter vector that has the same size as the parameter and is initialized to be zero
  MAKE_IN_PORT(in_avf_create_parameter, TParameter* (void), TThis);

  //!\brief clear a parameter vector (set zero)
  MAKE_IN_PORT(in_avf_zero_parameter, void (TParameter &outerparam), TThis);
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
  const TInt& out_episode_number_get (void) const {return mem_.EpisodeNumber;}
  const TSingleReward& out_return_in_episode_get (void) const {return return_in_episode_;}
  const TValue& out_td_error_get (void) const {return td_error_;}
  const TValue& out_current_action_value_get (void) const {return current_action_value_;}

  using TParent::signal_end_of_episode   ;
  using TParent::signal_execute_action   ;
  using TParent::signal_end_of_action    ;
  // using TParent::in_state                ;
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

  // GET_FROM_IN_PORT(avf_parameter_ref, const TParameter&, (void), ())
  // GET_FROM_IN_PORT(avf_parameter_val, void, (TParameter &outerparam), (outerparam))
  // GET_FROM_IN_PORT(avf_evaluate, void, (const TState &x, const TAction &a, TStateActionAttribute attrib), (x, a, attrib))
  // GET_FROM_IN_PORT(avf_greedy, void, (const TState &x, TAction *greedy, TStateActionAttribute attrib), (x, greedy, attrib))
  GET_FROM_IN_PORT(avf_select_action, void, (TAction *a, TStateActionAttribute attrib), (a,attrib))
  GET_FROM_IN_PORT(avf_replacing_trace, void, (TParameter &eligibility_trace), (eligibility_trace))
  GET_FROM_IN_PORT(avf_create_parameter, TParameter*, (void), ())
  GET_FROM_IN_PORT(avf_zero_parameter, void, (TParameter &outerparam), (outerparam))

  #undef GET_FROM_IN_PORT

  virtual bool is_updatable() const
    {
      if (in_updatable.ConnectionSize()==0)  return true;
      return in_updatable.GetFirst();
    }


  TAction               old_action;
  TAction               next_action;
  TValue                oldQ;  //!< assigned in startAction(), kept during the action
  TValue                oldV;  //!< assigned in startAction(), kept during the action
  TValue                nextQ;
  TValue                nextV;

  TParameterMemory      old_grad;  //!< previous gradient of the AVF at (old_state,old_action)
  TParameterMemory      next_grad;

  bool                  is_active_;
  TSingleReward         return_in_episode_;
  TInt                  actions_in_episode_;
  bool                  is_end_of_episode_;
  TValue                td_error_;
  TValue                current_action_value_;

  // temporary variable
  TParameterMemory      difference_;  //!< used in update

  // for activity trace
  TParameterMemory      activity_trace;  //!< activity trace


  inline TReal get_alpha (void) const;

  //! sub execution methods
  TValue   update_with_ql (const TSingleReward &R);
  TValue   update_with_sarsa (const TSingleReward &R);

  //! select \p TAgentTDGFA::next_action from real actions and internal actions
  virtual void     prepare_next_action ();

  //! execution methods
  virtual void     reset_episode (); //! execute this function to start new trial
  virtual TAction  select_action ();
  virtual void     update (const TSingleReward &reward);

};
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MTDGenericFuncApprox,TContinuousAction)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MTDGenericFuncApprox,TDiscreteAction)
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
} // end of tdgfa_tmpl_detail
//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_td_generic_fa_h
//-------------------------------------------------------------------------------------------
