//-------------------------------------------------------------------------------------------
/*! \file    avf_wire_fitting_smpl.h
    \brief   libskyai - wire-fitting function approximator module for action value function
              over continuous state-action space (simple version) (header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Dec.12, 2009-

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
#ifndef skyai_avf_wire_fitting_smpl_h
#define skyai_avf_wire_fitting_smpl_h
//-------------------------------------------------------------------------------------------
#include <skyai/interfaces/action_value_func.h>
#include <lora/octave_str.h>
#include <lora/variable_space_impl.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

namespace avf_wire_fitting_detail
{
  enum TAVFWFSmplActionSelection
  {
    asGreedy    =0,
    asEpsGreedy ,
    asWFBoltzman
  };
}
ENUM_STR_MAP_BEGIN_NS(avf_wire_fitting_detail, TAVFWFSmplActionSelection)
  ENUM_STR_MAP_ADD_NS(avf_wire_fitting_detail, asGreedy        )
  ENUM_STR_MAP_ADD_NS(avf_wire_fitting_detail, asEpsGreedy     )
  ENUM_STR_MAP_ADD_NS(avf_wire_fitting_detail, asWFBoltzman    )
ENUM_STR_MAP_END_NS  (avf_wire_fitting_detail, TAVFWFSmplActionSelection)
SPECIALIZE_TVARIABLE_TO_ENUM(avf_wire_fitting_detail::TAVFWFSmplActionSelection)

namespace avf_wire_fitting_detail
{
  enum TAVFWFSmplPolicyImprovement
  {
    piConst=0,    //!< not change the policy parameter
    piExpReduction   //!< exponential reduction
  };
}
ENUM_STR_MAP_BEGIN_NS(avf_wire_fitting_detail, TAVFWFSmplPolicyImprovement)
  ENUM_STR_MAP_ADD_NS(avf_wire_fitting_detail, piConst           )
  ENUM_STR_MAP_ADD_NS(avf_wire_fitting_detail, piExpReduction    )
ENUM_STR_MAP_END_NS  (avf_wire_fitting_detail, TAVFWFSmplPolicyImprovement)
SPECIALIZE_TVARIABLE_TO_ENUM(avf_wire_fitting_detail::TAVFWFSmplPolicyImprovement)

namespace avf_wire_fitting_detail
{
  enum TAVFWFSmplConstraintKind
  {
    sckNone    =0,
    sckMinMax
  };
}
ENUM_STR_MAP_BEGIN_NS(avf_wire_fitting_detail, TAVFWFSmplConstraintKind)
  ENUM_STR_MAP_ADD_NS(avf_wire_fitting_detail, sckNone         )
  ENUM_STR_MAP_ADD_NS(avf_wire_fitting_detail, sckMinMax       )
ENUM_STR_MAP_END_NS  (avf_wire_fitting_detail, TAVFWFSmplConstraintKind)
SPECIALIZE_TVARIABLE_TO_ENUM(avf_wire_fitting_detail::TAVFWFSmplConstraintKind)

namespace avf_wire_fitting_detail
{
  enum TAVFWFSmplActionNoiseKind
  {
    ankPolicyBased    =0,   // action = CtrlVec(wire) + (1.0-policy(wire)) * GaussianNoise
    ankTauBased             // action = CtrlVec(wire) + tau * GaussianNoise (tau is temperature param for Boltzmann selection)
  };
}
ENUM_STR_MAP_BEGIN_NS(avf_wire_fitting_detail, TAVFWFSmplActionNoiseKind)
  ENUM_STR_MAP_ADD_NS(avf_wire_fitting_detail, ankPolicyBased    )
  ENUM_STR_MAP_ADD_NS(avf_wire_fitting_detail, ankTauBased       )
ENUM_STR_MAP_END_NS  (avf_wire_fitting_detail, TAVFWFSmplActionNoiseKind)
SPECIALIZE_TVARIABLE_TO_ENUM(avf_wire_fitting_detail::TAVFWFSmplActionNoiseKind)


//-------------------------------------------------------------------------------------------
namespace avf_wire_fitting_detail
{
//-------------------------------------------------------------------------------------------


//===========================================================================================
//!\brief Configurations of MAVFWireFittingSimple
class TAVFWireFittingSimpleConfigurations
//===========================================================================================
{
public:

  // parameters of wire-fitting
  TReal                 SmoothingFactor;
  TReal                 Tiny;
  // parameters to make learning stable
  std::vector<TReal>    TraceMax;  /*!< (Used in replacing-trace) upper bound of each element of eligibility-trace.
                                        \note if TraceMax.size()==1: TraceMax[0] is used for every element of eligibility-trace,
                                              else if TraceMax.size()==dim(eligibility-trace): TraceMax[i] is used for i-th element of eligibility-trace
                                              otherwise: error  */

  int                   ActionDim;  //!< dimension of action

  // for action selection
  TContinuousAction     ActionMax, ActionMin;  //!< Upper and lower bound of action
  TAVFWFSmplActionSelection
                        ActionSelection;
  TAVFWFSmplPolicyImprovement
                        PolicyImprovement;  /*!< policy improvement method.
                                                \todo FIXME: it may be strange that the policy-improvement method
                                                  is provided by an action value function??? */
  TReal                 Eps;
  TReal                 EpsDecreasingFactor;  //!< used with piExpReduction. larger is decreasing faster (becoming greedy). do not set a value greater than 1.
  TReal                 Tau;
  TReal                 TauMin;   //!< lower bound of Tau
  TReal                 TauDecreasingFactor;  //!< used for piExpReduction. larger is decreasing faster (becoming greedy). do not set a value greater than 1.
  TReal                 NoiseFactor;  //!< used in ActionSelection=asWFBoltzman
  TReal                 MinimumNoiseVar;  //!< ditto
  TAVFWFSmplActionNoiseKind  ActionNoiseKind;  //!< ditto

  int                        WireSize;
  TAVFWFSmplConstraintKind   ConstraintKind;


  TAVFWireFittingSimpleConfigurations (var_space::TVariableMap &mmap)
    :
      SmoothingFactor        (1.0l),
      Tiny                   (0.01l),
      TraceMax               (1,1.0l),
      ActionDim              (1),
      ActionMax              (0),
      ActionMin              (0),
      ActionSelection        (asEpsGreedy),
      PolicyImprovement      (piConst),
      Eps                    (0.1l),
      EpsDecreasingFactor    (0.004l),
      Tau                    (1.0l),
      TauMin                 (DBL_TINY),
      TauDecreasingFactor    (0.002l),
      NoiseFactor            (0.0l),
      MinimumNoiseVar        (1.0e-100l),
      ActionNoiseKind        (ankPolicyBased),
      WireSize               (10),
      ConstraintKind         (sckMinMax)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( SmoothingFactor          );
      ADD( Tiny                     );
      ADD( TraceMax                 );
      ADD( ActionDim                );
      ADD( ActionMax                );
      ADD( ActionMin                );
      ADD( ActionSelection          );
      ADD( PolicyImprovement        );
      ADD( Eps                      );
      ADD( EpsDecreasingFactor      );
      ADD( Tau                      );
      ADD( TauMin                   );
      ADD( TauDecreasingFactor      );
      ADD( NoiseFactor              );
      ADD( MinimumNoiseVar          );
      ADD( ActionNoiseKind          );
      ADD( WireSize                 );
      ADD( ConstraintKind           );
      #undef ADD
    }

};
//-------------------------------------------------------------------------------------------


//===========================================================================================
class TAVFWireFittingSimpleParameter : public TActionValueFuncParamInterface
//===========================================================================================
{
protected:
public:

  std::vector<TRealVector>  Theta;
  std::vector<TRealVector>  CtrlVec;     //!< control input vector


  TAVFWireFittingSimpleParameter (void)  {}

  TAVFWireFittingSimpleParameter (var_space::TVariableMap &mmap)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( Theta       );
      ADD( CtrlVec     );
      #undef ADD
    }

  override ~TAVFWireFittingSimpleParameter(void) {}

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



  virtual void Clear (void)
    {
      Theta.clear();
      CtrlVec.clear();
    }

  /*!\brief initialize (all parameters are set to zero)
    \param  [in]bfsize   number of the basis functions
    \param  [in]ctrldim  number of the control dimension
    \param  [in]wiresize number of the wires */
  virtual void Init (int bfsize, int ctrldim, int wiresize)
    {
      Theta.resize(wiresize);
      for(std::vector<TRealVector>::iterator itr(Theta.begin()); itr!=Theta.end(); ++itr)
        itr->resize(bfsize);

      CtrlVec.resize(wiresize);
      for(std::vector<TRealVector>::iterator itr(CtrlVec.begin()); itr!=CtrlVec.end(); ++itr)
        itr->resize(ctrldim);

      Zero();
    }

  //!\brief number of the basis functions
  int BFSize (void) const {if(Theta.size()>0) return Theta.front().length(); else return 0;}

  //!\brief number of the control dimension
  int CtrlDim (void) const {if(CtrlVec.size()>0) return CtrlVec.front().length(); else return 0;}

  //!\brief number of the wires
  int WireSize (void) const {return Theta.size();}

  virtual int ParamSize (void) const
    {
      int n= WireSize() * (BFSize() + CtrlDim());
      return n;
    }

};
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief wire-fitting module to approximate an action value function (simple version)
    \note this AVF receives a feature vector (output of basis functions) as the state. */
class MAVFWireFittingSimple
    : public MActionValueFuncInterface <TRealVector, TContinuousAction>
//===========================================================================================
{
public:
  typedef MActionValueFuncInterface <
            TRealVector, TContinuousAction>  TParent;
  typedef MAVFWireFittingSimple              TThis;
  SKYAI_MODULE_NAMES(MAVFWireFittingSimple)

  MAVFWireFittingSimple (const std::string &v_instance_name)
    : TParent           (v_instance_name),
      conf_             (TParent::param_box_config_map()),
      param_            (TParent::param_box_memory_map()),
      in_feature        (*this),
      out_feature       (*this),
      in_episode_number (*this)
    {
      add_in_port (in_feature);
      add_out_port (out_feature);
      add_in_port (in_episode_number);
    }

  TAVFWireFittingSimpleParameter& Param()  {return param_;}
  const TAVFWireFittingSimpleParameter& Param() const {return param_;}

protected:

  TAVFWireFittingSimpleConfigurations conf_;
  // TAVFWireFittingSimpleMemories param_;
  TAVFWireFittingSimpleParameter param_;

  //!\brief input a feature vector (output of basis functions) by this port
  MAKE_IN_PORT(in_feature, const TRealVector& (void), TThis);

  //!\brief forward the in_feature
  MAKE_OUT_PORT(out_feature, const TRealVector&, (void), (), TThis);

  MAKE_IN_PORT(in_episode_number, const TInt& (void), TThis);

  override void slot_initialize_exec (void);
  override void slot_reset_exec (void);
  override void slot_add_to_parameter_exec (const TParameter &diff);

  override const TParameter& out_parameter_ref_get (void) const;
  override void out_parameter_val_get (TParameter &outerparam) const;
  override void out_evaluate_get (const TState &x, const TAction &a, TStateActionAttribute attrib) const;
  override void out_greedy_get (const TState &x, TAction *greedy, TStateActionAttribute attrib) const;
  override void out_select_action_get (TAction *a, TStateActionAttribute attrib) const;

  //!\warning Replacing eligibility trace for wire fitting is very experimental! We recommend not to use it.
  override void out_replacing_trace_get (TParameter &eligibility_trace) const;

  override TParameter* out_create_parameter_get (void) const;
  override void out_zero_parameter_get (TParameter &outerparam) const;

  virtual const TRealVector& out_feature_get(void) const {return get_feature();}

  #define GET_FROM_IN_PORT(x_in,x_return_type,x_arg_list,x_param_list)                          \
    x_return_type  get_##x_in x_arg_list const                                                  \
      {                                                                                         \
        if (in_##x_in.ConnectionSize()==0)                                                      \
          {LERROR("in "<<ModuleUniqueCode()<<", in_" #x_in " must be connected."); lexit(df);}  \
        return in_##x_in.GetFirst x_param_list;                                                 \
      }

  GET_FROM_IN_PORT(feature, const TRealVector&, (void), ())

  GET_FROM_IN_PORT(episode_number, const TInt&, (void), ())

  #undef GET_FROM_IN_PORT


  // temporary variables:

  mutable ColumnVector              policy;  //!\todo FIXME: to use in multi-thread mode, we must lock this variable
  MultiDimNormalDistribution      gauss_noise;

  struct TCache // current
    {
      TRealVector                   phi;
      std::vector<TRealVector>      u_i;
      std::vector<TValue>           q_i;
      TValue                        max_q;
      int                           max_q_idx;

      TAction                       a;
      std::vector<TValue>           D_i;
    };
  mutable TCache  cache;  //!\todo FIXME: for using in multi-thread mode, we must lock this variable


  // internal functions:

  void init_gauss_noise ();

  //! update cache for cache.phi \note REQUIRED: cache.phi
  void cache_update_phi () const;

  //! update cache for cache.a \note REQUIRED: cache.a , execute cache_update_phi
  void cache_update_a () const;

  //! select action from cache data and store it into cache.a \note REQUIRED: cache_update_phi
  void cache_select_action (TAVFWFSmplActionSelection ActionSelection, bool *res_is_greedy) const;

  //! calculate action value q from cache \note REQUIRED: cache_update_phi , cache_update_a
  void cache_calc_action_value (TReal &q) const;

  //! calculate gradient grad from cache \note REQUIRED: cache_update_phi , cache_update_a
  void cache_calc_gradient (TAVFWireFittingSimpleParameter &grad) const;

  //!\todo inefficient code:
  int get_feature_dim (void) const  {return get_feature().length();}

  void apply_min_max_constraint_to_des_i (void);

  inline TReal get_eps () const;
  inline TReal get_tau () const;

};  // end of MAVFWireFittingSimple
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of avf_wire_fitting_detail
//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_avf_wire_fitting_smpl_h
//-------------------------------------------------------------------------------------------
