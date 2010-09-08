//-------------------------------------------------------------------------------------------
/*! \file    avf_organizer.h
    \brief   libskyai - function approximator organizer module for action value function (header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Dec.21, 2009-
    \date    May.14, 2010

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
#ifndef skyai_avf_organizer_h
#define skyai_avf_organizer_h
//-------------------------------------------------------------------------------------------
#include <skyai/interfaces/action_value_func.h>
#include <vector>
// #include <lora/octave_str.h>
// #include <lora/vector_wrapper.h>
// #include <lora/vector_wrapper_oct.h>
#include <skyai/modules_std/bits/discaction_selection.h>
#include <lora/string_list_ext.h>
#include <lora/stl_ext.h>
#include <lora/variable_space_impl.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

namespace avf_organizer_detail
{
  enum TAVFOrganizerPolicyImprovement
  {
    piConst=0,    //!< not change the policy parameter
    piExpReduction   //!< exponential reduction
  };
}
ENUM_STR_MAP_BEGIN_NS(avf_organizer_detail, TAVFOrganizerPolicyImprovement)
  ENUM_STR_MAP_ADD_NS(avf_organizer_detail, piConst           )
  ENUM_STR_MAP_ADD_NS(avf_organizer_detail, piExpReduction    )
ENUM_STR_MAP_END_NS  (avf_organizer_detail, TAVFOrganizerPolicyImprovement)
SPECIALIZE_TVARIABLE_TO_ENUM(avf_organizer_detail::TAVFOrganizerPolicyImprovement)


//-------------------------------------------------------------------------------------------
namespace avf_organizer_detail
{
//-------------------------------------------------------------------------------------------


//===========================================================================================
//!\brief Configurations of MAVFOrganizer
class TAVFOrganizerConfigurations
//===========================================================================================
{
public:

  // for action selection
  disc_action::TActionSelection
                        ActionSelection;
  TAVFOrganizerPolicyImprovement
                        PolicyImprovement;  /*!< policy improvement method.
                                                \todo FIXME: it may be strange that the policy-improvement method
                                                  is provided by an action value function??? */
  TReal                 Eps;
  TReal                 EpsDecreasingFactor;  //!< used with piExpReduction. larger is decreasing faster (becoming greedy). do not set a value greater than 1.
  TReal                 Tau;
  TReal                 TauMin;   //!< lower bound of Tau
  TReal                 TauDecreasingFactor;  //!< used for piExpReduction. larger is decreasing faster (becoming greedy). do not set a value greater than 1.

  // for organization
  typedef  std::map<TString, TInt>  TOrderMap;
  TOrderMap             OrderOfLower;  //!< map of (instance name of lower module -> index). index starts from 0
  TOrderMap             OrderOfController;  //!< map of (unique name of exec. port of controller -> index).

  typedef  std::map<TString, TString>  TStrStrMap;
  TStrStrMap     AvailableSituations;  //!< map of (instance name of lower module -> situations where the lower module is available)

  typedef  std::map<TString, TReal>  THBWeightMap;
  THBWeightMap          HBWeight;  //!< weight for heuristic-Boltzmann action selection method. \note this is not good implementation because HBWeight depends on a `situation'

  TAVFOrganizerConfigurations (var_space::TVariableMap &mmap)
    :
      ActionSelection          (disc_action::asBoltzman),
      PolicyImprovement        (piConst),
      Eps                      (0.1l),
      EpsDecreasingFactor      (0.004l),
      Tau                      (1.0l),
      TauMin                   (DBL_TINY),
      TauDecreasingFactor      (0.002l)
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
      ADD( OrderOfLower           );
      ADD( OrderOfController      );
      ADD( AvailableSituations    );
      ADD( HBWeight               );
      #undef ADD
    }

};
//-------------------------------------------------------------------------------------------


//===========================================================================================
class TAVFOrganizerParameter : public TActionValueFuncParamInterface
//===========================================================================================
{
protected:

  struct TLowerParameter
    {
      bool IsZero;
      TActionValueFuncParamMemory Param;
    };
  typedef std::vector<TLowerParameter>   TLowerParamSet;

  TLowerParamSet  param_set_;

public:

  TAVFOrganizerParameter (void)  {}

  override ~TAVFOrganizerParameter(void) {}

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
      param_set_.clear();
    }

  int Size() const {return param_set_.size();}

  virtual void Resize (int size)
    {
      param_set_.resize(size);
    }

  bool  IsZero(int i) const {return param_set_[i].IsZero;}
  const TActionValueFuncParamMemory&  Param(int i) const {return param_set_[i].Param;}

  void  SetZero(int i)  {param_set_[i].Param().Zero(); param_set_[i].IsZero=true;}

  TActionValueFuncParamMemory&  SetParam(int i)  {param_set_[i].IsZero=false; return param_set_[i].Param;}

};
//-------------------------------------------------------------------------------------------



//===========================================================================================
/*!\brief Action-value-function organizer that organizes (structures) lower AVF modules */
class MAVFOrganizer
    : public MActionValueFuncInterface <TCompositeState, TCompositeAction>
//===========================================================================================
{
public:
  typedef MActionValueFuncInterface <
            TCompositeState, TCompositeAction>  TParent;
  typedef MAVFOrganizer                         TThis;
  SKYAI_MODULE_NAMES(MAVFOrganizer)

  MAVFOrganizer (const std::string &v_instance_name)
    : TParent           (v_instance_name),
      conf_             (TParent::param_box_config_map()),
      out_composite_state          (*this),
      slot_execute_composite_action(*this),
      in_episode_number            (*this),
      in_situation                 (*this),

      signal_avf_add_to_parameter  (*this),
      in_avf_state_d               (*this),
      in_avf_state_c               (*this),
      // in_avf_parameter_ref      (*this),
      // in_avf_parameter_val      (*this),
      in_avf_evaluate_dd           (*this),
      in_avf_evaluate_dc           (*this),
      in_avf_evaluate_cd           (*this),
      in_avf_evaluate_cc           (*this),
      in_avf_greedy_dd             (*this),
      in_avf_greedy_dc             (*this),
      in_avf_greedy_cd             (*this),
      in_avf_greedy_cc             (*this),
      in_avf_select_action_d       (*this),
      in_avf_select_action_c       (*this),
      in_avf_replacing_trace       (*this),
      in_avf_create_parameter      (*this),
      in_avf_zero_parameter        (*this),
      signal_execute_action_d      (*this),
      signal_execute_action_c      (*this)
    {
      add_out_port (out_composite_state);
      add_slot_port (slot_execute_composite_action);
      add_in_port (in_episode_number);
      add_in_port (in_situation);

      add_signal_port (signal_avf_add_to_parameter  );
      add_in_port (in_avf_state_d                   );
      add_in_port (in_avf_state_c                   );
      add_in_port (in_avf_evaluate_dd               );
      add_in_port (in_avf_evaluate_dc               );
      add_in_port (in_avf_evaluate_cd               );
      add_in_port (in_avf_evaluate_cc               );
      add_in_port (in_avf_greedy_dd                 );
      add_in_port (in_avf_greedy_dc                 );
      add_in_port (in_avf_greedy_cd                 );
      add_in_port (in_avf_greedy_cc                 );
      add_in_port (in_avf_select_action_d           );
      add_in_port (in_avf_select_action_c           );
      add_in_port (in_avf_replacing_trace           );
      add_in_port (in_avf_create_parameter          );
      add_in_port (in_avf_zero_parameter            );
      add_signal_port (signal_execute_action_d      );
      add_signal_port (signal_execute_action_c      );
    }

protected:

  TAVFOrganizerConfigurations conf_;

  // note: MAVFOrganizer does not have memory since every memory of lower modules is saved in each lower module.

  /*!\brief return a current composite state \p x
      \note x.DiscSet[0] is in_situation.GetFirst() */
  MAKE_OUT_PORT(out_composite_state, const TCompositeState&, (void), (), TThis);

  /*!\brief execute a composite action \p a
      \note action \p a must be
        {a.DiscSet[0]:lower-module-index,
         a.DiscSet[1]:discrete-action (if the lower module is discrete action type),
         a.ContSet[0]:continuous-action (if the lower module is continuous action type)} */
  MAKE_SLOT_PORT(slot_execute_composite_action, void, (const TCompositeAction &a), (a), TThis);

  MAKE_IN_PORT(in_episode_number, const TInt& (void), TThis);

  // //! input [module-index]={true,false} (true: the module is active in the current state)
  // MAKE_IN_PORT(in_available_modules, const std::vector<bool>& (void), TThis);
  MAKE_IN_PORT(in_situation, const TInt& (void), TThis);


  // [-- ports for lower function approximators
  MAKE_SIGNAL_PORT(signal_avf_add_to_parameter, void (const TParameter &), TThis);

  // //!\brief output a reference to the parameter
  // MAKE_IN_PORT_SPECIFIC(in_avf_parameter_ref, const TParameter& (void), TThis, SKYAI_CONNECTION_SIZE_MAX);

  MAKE_IN_PORT_SPECIFIC(in_avf_state_d, const TDiscreteState& (void), TThis, SKYAI_CONNECTION_SIZE_MAX);
  MAKE_IN_PORT_SPECIFIC(in_avf_state_c, const TContinuousState& (void), TThis, SKYAI_CONNECTION_SIZE_MAX);

  // //!\brief assign the parameter to outerparam
  // MAKE_IN_PORT_SPECIFIC(in_avf_parameter_val, void (TParameter &outerparam), TThis, SKYAI_CONNECTION_SIZE_MAX);

  MAKE_IN_PORT_SPECIFIC(in_avf_evaluate_dd, void (const TDiscreteState &x, const TDiscreteAction &a, TStateActionAttribute attrib), TThis, SKYAI_CONNECTION_SIZE_MAX);
  MAKE_IN_PORT_SPECIFIC(in_avf_evaluate_dc, void (const TDiscreteState &x, const TContinuousAction &a, TStateActionAttribute attrib), TThis, SKYAI_CONNECTION_SIZE_MAX);
  MAKE_IN_PORT_SPECIFIC(in_avf_evaluate_cd, void (const TContinuousState &x, const TDiscreteAction &a, TStateActionAttribute attrib), TThis, SKYAI_CONNECTION_SIZE_MAX);
  MAKE_IN_PORT_SPECIFIC(in_avf_evaluate_cc, void (const TContinuousState &x, const TContinuousAction &a, TStateActionAttribute attrib), TThis, SKYAI_CONNECTION_SIZE_MAX);

  //!\brief calculate a greedy action at x, assign it into *greedy (if not NULL), assign its action value into q (if not NULL)
  MAKE_IN_PORT_SPECIFIC(in_avf_greedy_dd, void (const TDiscreteState &x, TDiscreteAction *greedy, TStateActionAttribute attrib), TThis, SKYAI_CONNECTION_SIZE_MAX);
  MAKE_IN_PORT_SPECIFIC(in_avf_greedy_dc, void (const TDiscreteState &x, TContinuousAction *greedy, TStateActionAttribute attrib), TThis, SKYAI_CONNECTION_SIZE_MAX);
  MAKE_IN_PORT_SPECIFIC(in_avf_greedy_cd, void (const TContinuousState &x, TDiscreteAction *greedy, TStateActionAttribute attrib), TThis, SKYAI_CONNECTION_SIZE_MAX);
  MAKE_IN_PORT_SPECIFIC(in_avf_greedy_cc, void (const TContinuousState &x, TContinuousAction *greedy, TStateActionAttribute attrib), TThis, SKYAI_CONNECTION_SIZE_MAX);

  /*!\brief select an action at the current state by the current polocy
      \param [out]a  : if not NULL, selected action is stored
      \param [out]attrib  : the action value, the state value, and the gradient are stored */
  MAKE_IN_PORT_SPECIFIC(in_avf_select_action_d, void (TDiscreteAction *a, TStateActionAttribute attrib), TThis, SKYAI_CONNECTION_SIZE_MAX);
  MAKE_IN_PORT_SPECIFIC(in_avf_select_action_c, void (TContinuousAction *a, TStateActionAttribute attrib), TThis, SKYAI_CONNECTION_SIZE_MAX);

  //!\brief apply a replacing trace to an eligibility trace eligibility_trace
  MAKE_IN_PORT_SPECIFIC(in_avf_replacing_trace, void (TParameter &eligibility_trace), TThis, SKYAI_CONNECTION_SIZE_MAX);

  //!\brief create (allocate) a parameter vector that has the same size as the parameter and is initialized to be zero
  MAKE_IN_PORT_SPECIFIC(in_avf_create_parameter, TParameter* (void), TThis, SKYAI_CONNECTION_SIZE_MAX);

  //!\brief clear a parameter vector (set zero)
  MAKE_IN_PORT_SPECIFIC(in_avf_zero_parameter, void (TParameter &outerparam), TThis, SKYAI_CONNECTION_SIZE_MAX);

  MAKE_SIGNAL_PORT(signal_execute_action_d, void(const TDiscreteAction&), TThis);
  MAKE_SIGNAL_PORT(signal_execute_action_c, void(const TContinuousAction&), TThis);
  // ports for lower function approximators --]



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

  virtual const TCompositeState& out_composite_state_get (void) const;
  virtual void slot_execute_composite_action_exec (const TCompositeAction &a);


  #define GET_FROM_IN_PORT(x_in,x_return_type,x_arg_list,x_param_list)                          \
    x_return_type  get_##x_in x_arg_list const                                                  \
      {                                                                                         \
        if (in_##x_in.ConnectionSize()==0)                                                      \
          {LERROR("in "<<ModuleUniqueCode()<<", in_" #x_in " must be connected."); lexit(df);}  \
        return in_##x_in.GetFirst x_param_list;                                                 \
      }

  GET_FROM_IN_PORT(episode_number, const TInt&, (void), ())

  GET_FROM_IN_PORT(situation, const TInt&, (void), ())

  #undef GET_FROM_IN_PORT


  // temporary variables:

  mutable TCompositeState  tmp_state_;   //!< \todo FIXME: to use in multi-thread mode, we must lock this variable
  // mutable TRealVector  next_feature;  //!< \todo FIXME: to use in multi-thread mode, we must lock this variable
  mutable TRealVector  nextQs;        //!< \todo FIXME: to use in multi-thread mode, we must lock this variable
  mutable TRealVector  nextVs;        //!< ditto
  mutable TRealVector  next_policy;   //!< ditto

  enum TStateActionType {satUnknown=0, satDiscrete, satContinuous};

  // struct TExtAction
    // {
      // TDiscreteAction    Index;
      // TInt               LowerIndex;
      // TStateActionType   ActionType;
      // TDiscreteAction    Disc;
      // TContinuousAction  Cont;
      // TExtAction() : Index(-1) {}
    // };
  // mutable TExtAction next_action_;  //!<\todo (in the future) extend it to a ring buffer (cache more than 1 actions)

  struct TSelectActionInfo
    {
      TDiscreteAction              Disc;
      TContinuousAction            Cont;
      TValue                       Q;
      TValue                       V;
      TActionValueFuncParamMemory  Grad;
    };
  mutable std::vector<int>   cache_available_mods_;
  mutable std::vector<TSelectActionInfo> cache_select_info_;
  mutable std::vector<TReal>  cache_available_hbweights_;

  template <typename t_type, t_type init_value=0>
  struct IntInitializer
    {
      t_type Entity;
      operator t_type& ()  {return Entity;}
      operator const t_type& () const {return Entity;}
      IntInitializer(void) : Entity(init_value) {}
      IntInitializer(const t_type &i) : Entity(i) {}
    };

  struct TLowerConnection
    {
      const TThis *Outer;
      TString LowerModuleName;
      TStateActionType   ActionType;
      TStateActionType   StateType;
      std::vector<bool>  AvailableSituations;
      TReal         HBWeight;

      void SetActionType (TStateActionType a)
        {
          if (ActionType!=satUnknown && a!=ActionType)
            {LERROR(Outer->ModuleUniqueCode()<<": invalid lower module connections"
              " (ports for discrete and continuous action spaces are mixed)"); lexit(df);}
          ActionType= a;
        }
      void SetStateType (TStateActionType a)
        {
          if (StateType!=satUnknown && a!=StateType)
            {LERROR(Outer->ModuleUniqueCode()<<": invalid lower module connections"
              " (ports for discrete and continuous state spaces are mixed)"); lexit(df);}
          StateType= a;
        }

      #define DEF_ITR(x_port)  \
        GET_PORT_TYPE(x_port)::TConnectedPortIterator  x_port##_citr_entity ;                     \
        IntInitializer<bool,false>                     x_port##_connected ;                       \
        const GET_PORT_TYPE(x_port)::TConnectedPortIterator& x_port##_citr () const               \
          {                                                                                       \
            if (!x_port##_connected)                                                              \
              {LERROR(Outer->ModuleUniqueCode()<<" requires that a port related to the lower module "  \
                <<LowerModuleName<<" is connected to the port "<<#x_port); lexit(df);}                 \
            return x_port##_citr_entity ;                                                              \
          }                                                                                       \
        void  x_port##_citr (GET_PORT_TYPE(x_port)::TConnectedPortIterator citr)                  \
          {                                                                                       \
            x_port##_citr_entity = citr;                                                          \
            x_port##_connected = true;                                                            \
          }
      DEF_ITR(signal_avf_add_to_parameter      )
      DEF_ITR(in_avf_state_d                   )
      DEF_ITR(in_avf_state_c                   )
      // DEF_ITR(in_avf_parameter_ref          )
      // DEF_ITR(in_avf_parameter_val          )
      DEF_ITR(in_avf_evaluate_dd               )
      DEF_ITR(in_avf_evaluate_dc               )
      DEF_ITR(in_avf_evaluate_cd               )
      DEF_ITR(in_avf_evaluate_cc               )
      DEF_ITR(in_avf_greedy_dd                 )
      DEF_ITR(in_avf_greedy_dc                 )
      DEF_ITR(in_avf_greedy_cd                 )
      DEF_ITR(in_avf_greedy_cc                 )
      DEF_ITR(in_avf_select_action_d           )
      DEF_ITR(in_avf_select_action_c           )
      DEF_ITR(in_avf_replacing_trace           )
      DEF_ITR(in_avf_create_parameter          )
      DEF_ITR(in_avf_zero_parameter            )
      DEF_ITR(signal_execute_action_d          )
      DEF_ITR(signal_execute_action_c          )
      #undef DEF_ITR

      TLowerConnection() : Outer(NULL), ActionType(satUnknown), StateType(satUnknown)  {}
    };
  typedef std::vector<TLowerConnection> TLowerCMap;
  TLowerCMap  lowerc_map_;
  TInt disc_state_lowerc_size_;
  TInt cont_state_lowerc_size_;
  std::vector<TInt>  lower_idx_to_state_idx_;


  // internal functions:

  void make_lowerc_map (void);
  void init_parameter (TAVFOrganizerParameter &outerparam, bool force_create=false) const;

  inline TReal get_eps () const;
  inline TReal get_tau () const;

};  // end of MAVFOrganizer
//-------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------
}  // end of avf_organizer_detail
//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_avf_organizer_h
//-------------------------------------------------------------------------------------------
