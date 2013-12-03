//-------------------------------------------------------------------------------------------
/*! \file    avf_linear_discaction_b.h
    \brief   libskyai - batch-update version of linear function approximator module for action value function over discrete action space (header)
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
#ifndef skyai_avf_linear_discaction_b_h
#define skyai_avf_linear_discaction_b_h
//-------------------------------------------------------------------------------------------
#include <skyai/interfaces/action_value_func.h>
#include <skyai/modules_std/bits/discaction_selection.h>
#include <lora/octave_str.h>
#include <lora/variable_space_impl.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------


namespace avf_linear_discaction_b_detail
{
  enum TAVFLinearDABPolicyImprovement
  {
    piConst=0,    //!< not change the policy parameter
    piExpReduction   //!< exponential reduction
  };
}
ENUM_STR_MAP_BEGIN_NS(avf_linear_discaction_b_detail, TAVFLinearDABPolicyImprovement)
  ENUM_STR_MAP_ADD_NS(avf_linear_discaction_b_detail, piConst           )
  ENUM_STR_MAP_ADD_NS(avf_linear_discaction_b_detail, piExpReduction    )
ENUM_STR_MAP_END_NS  (avf_linear_discaction_b_detail, TAVFLinearDABPolicyImprovement)
SPECIALIZE_TVARIABLE_TO_ENUM(avf_linear_discaction_b_detail::TAVFLinearDABPolicyImprovement)


//-------------------------------------------------------------------------------------------
namespace avf_linear_discaction_b_detail
{
//-------------------------------------------------------------------------------------------


//===========================================================================================
//!\brief Configurations of MAVFLinearDiscActionB
class TAVFLinearDiscActionBConfigurations
//===========================================================================================
{
public:

  // for action selection
  disc_action::TActionSelection
                        ActionSelection;
  TAVFLinearDABPolicyImprovement
                        PolicyImprovement;  /*!< policy improvement method.
                                                \todo FIXME: it may be strange that the policy-improvement method
                                                  is provided by an action value function??? */
  TReal                 Eps;
  TReal                 EpsDecreasingFactor;  //!< used with piExpReduction. larger is decreasing faster (becoming greedy). do not set a value greater than 1.
  TReal                 Tau;
  TReal                 TauMin;   //!< lower bound of Tau
  TReal                 TauDecreasingFactor;  //!< used for piExpReduction. larger is decreasing faster (becoming greedy). do not set a value greater than 1.

  TReal                 RegularizationCoefficient;  //!< used for regularized least squares

  TAVFLinearDiscActionBConfigurations (var_space::TVariableMap &mmap)
    :
      ActionSelection        (disc_action::asBoltzman),
      PolicyImprovement      (piConst),
      Eps                    (0.1l),
      EpsDecreasingFactor    (0.004l),
      Tau                    (1.0l),
      TauMin                 (DBL_TINY),
      TauDecreasingFactor    (0.002l),
      RegularizationCoefficient (0.01l)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( ActionSelection        );
      ADD( PolicyImprovement      );
      ADD( Eps                    );
      ADD( EpsDecreasingFactor    );
      ADD( Tau                    );
      ADD( TauMin                 );
      ADD( TauDecreasingFactor    );
      ADD( RegularizationCoefficient );
      #undef ADD
    }

};
//-------------------------------------------------------------------------------------------


//===========================================================================================
class TAVFLinearDiscActionBParameter
//===========================================================================================
{
protected:
public:

  //! Theta[0,..,NA-1],  Theta[a][0,..,NK], NA: number of action, NK: number of basis functions
  std::vector<TRealVector>  Theta;

  TAVFLinearDiscActionBParameter (void)  {}

  TAVFLinearDiscActionBParameter (var_space::TVariableMap &mmap)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( Theta  );
      #undef ADD
    }

  /*!\brief initialize (all parameters are set to zero)
    \param  [in]bfsize   number of the basis functions
    \param  [in]action_set_size  size of the action set */
  void Init (int bfsize, int action_set_size)
    {
      Theta.resize(action_set_size);
      for(std::vector<TRealVector>::iterator itr(Theta.begin()); itr!=Theta.end(); ++itr)
      {
        GenResize(*itr, bfsize);
        SetZero(*itr);
      }
    }

  //!\brief number of the basis functions
  int BFSize (void) const {if(Theta.size()>0) return Theta.front().length(); else return 0;}

  //!\brief size of the action set
  int ActionSetSize (void) const {return Theta.size();}

};
//-------------------------------------------------------------------------------------------



//===========================================================================================
/*!\brief batch-update version of linear-model module to approximate an action value function over a continuous state and a discrete action space
    \note this AVF receives a feature vector (output of basis functions) as the state.
          i.e. TState == TFeature
    \todo This module has many codes which are the same as MAVFLinearDiscAction, so create a base class or inherit the other module. */
class MAVFLinearDiscActionB
    : public MActionValueFuncInterface <TRealVector, TDiscreteAction>
//===========================================================================================
{
public:
  typedef MActionValueFuncInterface <
            TRealVector, TDiscreteAction> TParent;
  typedef MAVFLinearDiscActionB           TThis;
  SKYAI_MODULE_NAMES(MAVFLinearDiscActionB)

  MAVFLinearDiscActionB (const std::string &v_instance_name)
    : TParent           (v_instance_name),
      conf_             (TParent::param_box_config_map()),
      param_            (TParent::param_box_memory_map()),
      mutable_theta_    (&param_.Theta),
      in_feature              (*this),
      out_feature             (*this),
      in_episode_number       (*this),
      in_action_set_size      (*this),
      in_action_availability  (*this),
      in_action_availability_s(*this),
      out_avtable             (*this),
      out_settable_avtable    (*this)
    {
      add_in_port (in_feature);
      add_out_port (out_feature);
      add_in_port (in_episode_number);
      add_in_port (in_action_set_size);
      add_in_port (in_action_availability);
      add_in_port (in_action_availability_s);
      add_out_port (out_avtable);
      add_out_port (out_settable_avtable);
    }

  TAVFLinearDiscActionBConfigurations& Config()  {return conf_;}
  const TAVFLinearDiscActionBConfigurations& Config() const {return conf_;}

  TAVFLinearDiscActionBParameter& Param()  {return param_;}
  const TAVFLinearDiscActionBParameter& Param() const {return param_;}

protected:

  TAVFLinearDiscActionBConfigurations conf_;
  TAVFLinearDiscActionBParameter param_;

  mutable TRealVectorSet *mutable_theta_;  // defined only for out_settable_avtable_get


  //!\brief input a feature vector (output of basis functions) by this port
  MAKE_IN_PORT(in_feature, const TRealVector& (void), TThis);

  //!\brief forward the in_feature
  MAKE_OUT_PORT(out_feature, const TRealVector&, (void), (), TThis);

  MAKE_IN_PORT(in_episode_number, const TInt& (void), TThis);

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

  /*!\brief update the parameter from the training sample data. */
  override void slot_train_with_data_exec (const std::list<TActionValueFuncTrainingSample<TState,TAction> > &data);

  override void out_evaluate_get (const TState &x, const TAction &a, TStateActionAttribute attrib) const;
  override void out_greedy_get (const TState &x, TAction *greedy, TStateActionAttribute attrib) const;
  override void out_select_action_get (TAction *a, TStateActionAttribute attrib) const;

  const TRealVector& out_feature_get(void) const {return get_feature();}
  const TRealVectorSet& out_avtable_get(void) const {return param_.Theta;}
  TRealVectorSet& out_settable_avtable_get(void) const {return *mutable_theta_;}

  #define GET_FROM_IN_PORT(x_in,x_return_type,x_arg_list,x_param_list)                          \
    x_return_type  get_##x_in x_arg_list const                                                  \
      {                                                                                         \
        if (in_##x_in.ConnectionSize()==0)                                                      \
          {LERROR("in "<<ModuleUniqueCode()<<", in_" #x_in " must be connected."); lexit(df);}  \
        return in_##x_in.GetFirst x_param_list;                                                 \
      }

  GET_FROM_IN_PORT(feature, const TRealVector&, (void), ())

  GET_FROM_IN_PORT(episode_number, const TInt&, (void), ())

  GET_FROM_IN_PORT(action_set_size, const TInt&, (void), ())

  #undef GET_FROM_IN_PORT


  // temporary variables:

  mutable TRealVector  next_feature;  //!< \todo when using in multi-thread mode, we must lock this variable
  mutable TRealVector  nextQs;        //!< ditto
  mutable TRealVector  next_policy;   //!< ditto


  // internal functions:

  //!\todo inefficient code:
  int get_feature_dim (void) const  {return get_feature().length();}

  inline TReal get_eps () const;
  inline TReal get_tau () const;

};  // end of MAVFLinearDiscActionB
//-------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------
}  // end of avf_linear_discaction_b_detail
//-------------------------------------------------------------------------------------------
}  // end of loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_avf_linear_discaction_b_h
//-------------------------------------------------------------------------------------------
