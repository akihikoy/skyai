//-------------------------------------------------------------------------------------------
/*! \file    avf_table.h
    \brief   libskyai - table-lookup function approximator module for action value function over discrete state-action space (header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Jul.12, 2012

    Copyright (C) 2012  Akihiko Yamaguchi

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
#ifndef skyai_avf_table_h
#define skyai_avf_table_h
//-------------------------------------------------------------------------------------------
#include <skyai/interfaces/action_value_func.h>
#include <skyai/modules_std/bits/discaction_selection.h>
#include <skyai/modules_std/avf_linear_discaction.h>
#include <lora/octave_str.h>
#include <lora/variable_space_impl.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------
namespace avf_table_detail
{
//-------------------------------------------------------------------------------------------


#if 0
//===========================================================================================
class TAVFTableParameter : public TActionValueFuncParamInterface
//===========================================================================================
{
protected:
public:

  //! Theta[0,..,NA-1],  Theta[a][0,..,NK], NA: number of action, NK: number of state
  std::vector<TRealVector>  Theta;

  TAVFTableParameter (void)  {}

  TAVFTableParameter (var_space::TVariableMap &mmap)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( Theta  );
      #undef ADD
    }

  override ~TAVFTableParameter(void) {}

  //!\brief assign zero (but size is not changed)
  override void Zero (void);

  //!\brief return the norm of the parameter
  override TReal Norm (void) const;

  /*!\brief return (*this = rhs) */
  override const TActionValueFuncParamInterface& operator= (const TActionValueFuncParamInterface &rhs);

  /*!\brief return (*this += rhs) */
  override const TActionValueFuncParamInterface& operator+= (const TActionValueFuncParamInterface &rhs);

  //!\brief return (*this += weight*rhs)
  override const TActionValueFuncParamInterface& AddProd (const TReal &weight, const TActionValueFuncParamInterface &rhs);

  /*!\brief return (*this *= rhs) */
  override const TActionValueFuncParamInterface& operator*= (const TReal &rhs);


  /*!\brief initialize (all parameters are set to zero)
    \param  [in]state_set_size   size of the state set
    \param  [in]action_set_size  size of the action set */
  virtual void Init (int state_set_size, int action_set_size)
    {
      Theta.resize(action_set_size);
      for(std::vector<TRealVector>::iterator itr(Theta.begin()); itr!=Theta.end(); ++itr)
        GenResize(*itr, state_set_size);

      Zero();
    }

  //!\brief size of the state set
  int StateSetSize (void) const {if(Theta.size()>0) return Theta.front().length(); else return 0;}

  //!\brief size of the action set
  int ActionSetSize (void) const {return Theta.size();}

  virtual int ParamSize (void) const
    {
      int n= ActionSetSize() * StateSetSize();
      return n;
    }

};
//-------------------------------------------------------------------------------------------
#endif



//===========================================================================================
/*!\brief table-lookup function approximator for an action value function over a discrete state and a discrete action space */
class MAVFTable
    : public MActionValueFuncInterface <TDiscreteState, TDiscreteAction>
//===========================================================================================
{
public:
  typedef MActionValueFuncInterface <
            TDiscreteState, TDiscreteAction> TParent;
  typedef MAVFTable                          TThis;
  SKYAI_MODULE_NAMES(MAVFTable)

  typedef avf_linear_discaction_detail::TAVFLinearDiscActionConfigurations  TAVFTableConfigurations;
  //! use MAVFLinearDiscAction's param class where BFSize==state-set-size, feature==disc state
  typedef avf_linear_discaction_detail::TAVFLinearDiscActionParameter  TAVFTableParameter;

  MAVFTable (const std::string &v_instance_name)
    : TParent           (v_instance_name),
      conf_             (TParent::param_box_config_map()),
      param_            (TParent::param_box_memory_map()),
      mutable_theta_    (&param_.Theta),
      in_state                (*this),
      out_state               (*this),
      in_episode_number       (*this),
      in_state_set_size       (*this),
      in_action_set_size      (*this),
      in_action_availability  (*this),
      in_action_availability_s(*this),
      out_avtable             (*this),
      out_settable_avtable    (*this)
    {
      add_in_port (in_state);
      add_out_port (out_state);
      add_in_port (in_episode_number);
      add_in_port (in_state_set_size);
      add_in_port (in_action_set_size);
      add_in_port (in_action_availability);
      add_in_port (in_action_availability_s);
      add_out_port (out_avtable);
      add_out_port (out_settable_avtable);
    }

  TAVFTableConfigurations& Config()  {return conf_;}
  const TAVFTableConfigurations& Config() const {return conf_;}

  TAVFTableParameter& Param()  {return param_;}
  const TAVFTableParameter& Param() const {return param_;}

protected:

  TAVFTableConfigurations  conf_;
  TAVFTableParameter  param_;

  mutable TRealVectorSet *mutable_theta_;  // defined only for out_settable_avtable_get


  //!\brief input a state by this port
  MAKE_IN_PORT(in_state, const TDiscreteState& (void), TThis);

  //!\brief forward the in_state
  MAKE_OUT_PORT(out_state, const TDiscreteState&, (void), (), TThis);

  MAKE_IN_PORT(in_episode_number, const TInt& (void), TThis);

  MAKE_IN_PORT(in_state_set_size, const TInt& (void), TThis);

  MAKE_IN_PORT(in_action_set_size, const TInt& (void), TThis);

  /*!\brief input availability (for the current state) of each action in whole action set; e.g. (true,false,true,..,true)
      \note if this port is not connected, the whole action set is used */
  MAKE_IN_PORT(in_action_availability, const TBoolVector& (void), TThis);

  /*!\brief input availability (for the state x) of each action in whole action set; e.g. (true,false,true,..,true)
      \note if this port is not connected, the whole action set is used
      \note this in-port is used in out_evaluate_get and out_greedy_get. if use these ports,
            confirm that in_action_availability.GetFirst()==in_action_availability_s.GetFirst(in_feature.GetFirst()) */
  MAKE_IN_PORT(in_action_availability_s, const TBoolVector& (const TState &x), TThis);

  /*!\brief output parameter vector; action value table
      (Theta[0,..,NA-1],  Theta[a][0,..,NK], NA: number of action, NK: number of basis functions) */
  MAKE_OUT_PORT(out_avtable, const TRealVectorSet&, (void), (), TThis);

  /*!\brief output settable parameter vector; action value table
      (Theta[0,..,NA-1],  Theta[a][0,..,NK], NA: number of action, NK: number of basis functions) */
  MAKE_OUT_PORT(out_settable_avtable, TRealVectorSet&, (void), (), TThis);

  override void slot_initialize_exec (void);
  override void slot_reset_exec (void);
  override void slot_add_to_parameter_exec (const TParameter &diff);

  override const TParameter& out_parameter_ref_get (void) const;
  override void out_parameter_val_get (TParameter &outerparam) const;
  override void out_evaluate_get (const TState &x, const TAction &a, TStateActionAttribute attrib) const;
  override void out_greedy_get (const TState &x, TAction *greedy, TStateActionAttribute attrib) const;
  override void out_select_action_get (TAction *a, TStateActionAttribute attrib) const;
  override void out_replacing_trace_get (TParameter &eligibility_trace) const;

  override TParameter* out_create_parameter_get (void) const;
  override void out_zero_parameter_get (TParameter &outerparam) const;

  const TDiscreteState& out_state_get(void) const {return get_state();}
  const TRealVectorSet& out_avtable_get(void) const {return param_.Theta;}
  TRealVectorSet& out_settable_avtable_get(void) const {return *mutable_theta_;}

  #define GET_FROM_IN_PORT(x_in,x_return_type,x_arg_list,x_param_list)                          \
    x_return_type  get_##x_in x_arg_list const                                                  \
      {                                                                                         \
        if (in_##x_in.ConnectionSize()==0)                                                      \
          {LERROR("in "<<ModuleUniqueCode()<<", in_" #x_in " must be connected."); lexit(df);}  \
        return in_##x_in.GetFirst x_param_list;                                                 \
      }

  GET_FROM_IN_PORT(state, const TDiscreteState&, (void), ())

  GET_FROM_IN_PORT(episode_number, const TInt&, (void), ())

  GET_FROM_IN_PORT(state_set_size, const TInt&, (void), ())

  GET_FROM_IN_PORT(action_set_size, const TInt&, (void), ())

  #undef GET_FROM_IN_PORT


  // temporary variables:

  mutable TDiscreteState  next_state;  //!< \todo when using in multi-thread mode, we must lock this variable
  mutable TRealVector  nextQs;        //!< ditto
  mutable TRealVector  next_policy;   //!< ditto


  // internal functions:

  inline TReal get_eps () const;
  inline TReal get_tau () const;

};  // end of MAVFTable
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
}  // end of avf_table_detail
//-------------------------------------------------------------------------------------------
}  // end of loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_avf_table_h
//-------------------------------------------------------------------------------------------
