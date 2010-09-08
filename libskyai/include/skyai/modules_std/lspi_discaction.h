//-------------------------------------------------------------------------------------------
/*! \file    lspi_discaction.h
    \brief   libskyai - LSPI implementation for a discrete action set [Lagoudakis and Parr, 2001]
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Mar.29, 2010-

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

    -----------------------------------------------------------------------------------------

    \warning This module is highly experimental!
*/
//-------------------------------------------------------------------------------------------
#ifndef skyai_lspi_discaction_h
#define skyai_lspi_discaction_h
//-------------------------------------------------------------------------------------------
#include <skyai/modules_std/bits/discaction_selection.h>
#include <skyai/interfaces/behavior.h>
#include <skyai/types.h>
#include <octave/Sparse.h>
#include <octave/dSparse.h>
#include <lora/small_classes.h>
#include <lora/variable_space_impl.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

namespace lspi_discaction_detail
{
  enum TPolicyImprovement
    {
      piConst=0                 , //!< not change the policy parameter
      piExpReduction              //!< exponential reduction
    };
} // end of namespace lspi_discaction_detail
ENUM_STR_MAP_BEGIN_NS(lspi_discaction_detail, TPolicyImprovement)
  ENUM_STR_MAP_ADD_NS(lspi_discaction_detail, piConst                     )
  ENUM_STR_MAP_ADD_NS(lspi_discaction_detail, piExpReduction              )
ENUM_STR_MAP_END_NS  (lspi_discaction_detail, TPolicyImprovement)
SPECIALIZE_TVARIABLE_TO_ENUM(lspi_discaction_detail::TPolicyImprovement)

namespace lspi_discaction_detail
{
  enum TMatrixSolver
    {
      msSparse=0       , //!< using liboctave::SparseMatrix::solve (which uses libumfpack)
      msSVD              //!< using SVD \warning this requires huge memory for large feature vector
    };
} // end of namespace lspi_discaction_detail
ENUM_STR_MAP_BEGIN_NS(lspi_discaction_detail, TMatrixSolver)
  ENUM_STR_MAP_ADD_NS(lspi_discaction_detail, msSparse     )
  ENUM_STR_MAP_ADD_NS(lspi_discaction_detail, msSVD        )
ENUM_STR_MAP_END_NS  (lspi_discaction_detail, TMatrixSolver)
SPECIALIZE_TVARIABLE_TO_ENUM(lspi_discaction_detail::TMatrixSolver)

namespace lspi_discaction_detail
{
  enum TLSTDQKind
    {
      lkDirect=0       , //!< directly solve the inverse
      lkRecursive        //!< recursive solve the inverse
    };
} // end of namespace lspi_discaction_detail
ENUM_STR_MAP_BEGIN_NS(lspi_discaction_detail, TLSTDQKind)
  ENUM_STR_MAP_ADD_NS(lspi_discaction_detail, lkDirect      )
  ENUM_STR_MAP_ADD_NS(lspi_discaction_detail, lkRecursive   )
ENUM_STR_MAP_END_NS  (lspi_discaction_detail, TLSTDQKind)
SPECIALIZE_TVARIABLE_TO_ENUM(lspi_discaction_detail::TLSTDQKind)


//-------------------------------------------------------------------------------------------
namespace lspi_discaction_detail
{
//-------------------------------------------------------------------------------------------


typedef std::pair<TInt,TypeExtS<TRealVector>::value_type>  TFeatureElement;
typedef std::vector<TFeatureElement>                       TSparseFeature;

struct TLSPISample
{
  TSparseFeature  old_sfeature, next_sfeature;
  TDiscreteAction old_action;
  TSingleReward   reward;
  TLSPISample (const TSingleReward v_reward, const TDiscreteAction &v_old_action, const TRealVector &old_feature, const TRealVector &next_feature,
                const TReal &nonzero_feature_threshold, TInt nonzero_feature_max_size);
};

typedef TListMap<TInt,TLSPISample>  TLSPIData;
//-------------------------------------------------------------------------------------------


//===========================================================================================
//!\brief Configurations of MLSPI_TDiscreteAction
class TConfigurations
//===========================================================================================
{
public:

  TReal                 Gamma;
  TReal                 Eps, Tau;  //!< policy parameters
  TReal                 TauMin;   //!< lower bound of Tau
  TReal                 EpsDecreasingFactor, TauDecreasingFactor;  //!< used for piExpReduction. larger is decreasing faster (becoming greedy). do not set a value greater than 1.
  disc_action::TActionSelection
                        ActionSelection;      //!< defined in skyai/modules_std/bits/discaction_selection.h
  TPolicyImprovement
                        PolicyImprovement;     //!< policy improvement method

  TInt                  NonzeroFeatureSize;    //!< number of nonzero elements of a feature vector
  TInt                  NonzeroFeatureMaxSize;  //!< maximum number of nonzero elements of a feature vector
  TReal                 NonzeroFeatureThreshold;  //!< if an element of a feature vector is less than this value, it is assumed to be zero
  TInt                  MaxACapacity;       //!< maximum number of nonzer elements that the matrix A can include
  TInt                  MaxDataSizePerDim;  //!< maximum number of data per each element of a (complete) feature vector phi(s,a)

  TInt                  LSPICycle;  //!< at the end of every LSPICycle episodes, LSPI is executed
  TInt                  LSTDQIterations;  //!< number of iterations of LSTD-Q
  TMatrixSolver         MatrixSolver;
  TLSTDQKind            LSTDQKind;

  TConfigurations(var_space::TVariableMap &mmap)
    :
      Gamma             (0.9l),
      Eps               (0.1l),  // smaller is more greedy
      Tau               (1.0l),  // smaller is more greedy
      TauMin                  (DBL_TINY),
      EpsDecreasingFactor     (0.002l),
      TauDecreasingFactor     (0.002l),
      ActionSelection         (disc_action::asBoltzman),
      PolicyImprovement       (piExpReduction),
      NonzeroFeatureSize      (5),
      NonzeroFeatureMaxSize   (10),
      NonzeroFeatureThreshold (0.01),
      MaxACapacity            (5000*5000),
      MaxDataSizePerDim       (100),
      LSPICycle               (1),
      LSTDQIterations         (1),
      MatrixSolver            (msSparse),
      LSTDQKind               (lkDirect)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( Gamma                     );
      ADD( Eps                       );
      ADD( Tau                       );
      ADD( TauMin                    );
      ADD( EpsDecreasingFactor       );
      ADD( TauDecreasingFactor       );
      ADD( ActionSelection           );
      ADD( PolicyImprovement         );
      ADD( NonzeroFeatureSize        );
      ADD( NonzeroFeatureMaxSize     );
      ADD( NonzeroFeatureThreshold   );
      ADD( MaxACapacity              );
      ADD( MaxDataSizePerDim         );
      ADD( LSPICycle                 );
      ADD( LSTDQIterations           );
      ADD( MatrixSolver              );
      ADD( LSTDQKind                 );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------



//===========================================================================================
//!\brief Memories of MLSPI_TDiscreteAction
class TMemories
//===========================================================================================
{
public:

  /*! weight vector: [w_0, w_1, ..., w_{NA-1}], w_a= [w_{a,1}, w_{a,2},..., w_{a,NK}];
      NA: number of the actions, NK: number of the basis functions  */
  TRealVector       W;

  TInt  EpisodeNumber;  //!< episode number

  TMemories (var_space::TVariableMap &mmap)
    :
      EpisodeNumber(0)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( W  );
      ADD( EpisodeNumber );
      #undef ADD
    }

};
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief  LSPI module for a discrete action set [Lagoudakis and Parr, 2001]
    \note the function approximator is included in this module
    \note the linear function approximator: Q(x,a) = feature.transpose() * W[NK*a..NK*(a+1)-1]
      where feature is a feature vector of the basis functions over the state space,
      and W is the weight vector
    \todo TODO: implement out_td_error_get function
*/
class MLSPI_TDiscreteAction
    : public MRLLinearFuncApproxInterface_TDiscreteAction
//===========================================================================================
{
public:
  typedef MRLLinearFuncApproxInterface_TDiscreteAction  TParent;
  typedef TRealVector                                   TFeature; //!< type to represent the feature vector
  typedef MLSPI_TDiscreteAction                         TThis;
  SKYAI_MODULE_NAMES(MLSPI_TDiscreteAction)

  MLSPI_TDiscreteAction (const std::string &v_instance_name)
    :
      TParent                       (v_instance_name),
      conf_                         (TParent::param_box_config_map()),
      mem_                          (TParent::param_box_memory_map()),
      out_episode_number            (*this),
      out_return_in_episode         (*this),
      out_td_error                  (*this),
      out_current_action_value      (*this),
      out_state_value               (*this),
      out_action_value              (*this),

      old_action        (-1),
      next_action       (-1),
      is_active_        (false),
      return_in_episode_ (0.0l),
      actions_in_episode_(0),
      is_end_of_episode_(false)
    {
      add_out_port (out_episode_number);
      add_out_port (out_return_in_episode);
      add_out_port (out_td_error);
      add_out_port (out_current_action_value);
      add_out_port (out_state_value);
      add_out_port (out_action_value);
    }

  virtual ~MLSPI_TDiscreteAction() {}

protected:

  TConfigurations   conf_;
  TMemories         mem_;

  MAKE_OUT_PORT(out_episode_number, const TInt&, (void), (), TThis);
  MAKE_OUT_PORT(out_return_in_episode, const TSingleReward&, (void), (), TThis);
  MAKE_OUT_PORT(out_td_error, const TValue&, (void), (), TThis);
  MAKE_OUT_PORT(out_current_action_value, const TValue&, (void), (), TThis);

  MAKE_OUT_PORT(out_state_value, void, (const TFeature &phi, TValue &v), (phi,v), TThis);
  MAKE_OUT_PORT(out_action_value, void, (const TFeature &phi, TRealVector &v), (phi,v), TThis);

  override void slot_initialize_exec (void);
  override void slot_start_episode_exec (void);
  override void slot_finish_episode_exec (void);
  override void slot_finish_episode_immediately_exec (void);
  override void slot_finish_action_exec (void);
  virtual const TInt& out_episode_number_get (void) const {return mem_.EpisodeNumber;}
  virtual const TSingleReward& out_return_in_episode_get (void) const {return return_in_episode_;}
  virtual const TValue& out_td_error_get (void) const {return td_error_;}
  virtual const TValue& out_current_action_value_get (void) const {return current_action_value_;}
  virtual void out_state_value_get (const TFeature &phi, TValue &v) const;
  virtual void out_action_value_get (const TFeature &phi, TRealVector &v) const;

  using TParent::signal_end_of_episode   ;
  using TParent::signal_execute_action   ;
  using TParent::signal_end_of_action    ;
  using TParent::in_updatable            ;

  using TParent::in_action_set_size      ;
  using TParent::in_reward               ;
  using TParent::in_feature              ;

  using TParent::ModuleUniqueCode        ;

protected:

  #define GET_FROM_IN_PORT(x_in,x_return_type,x_arg_list,x_param_list)                          \
    x_return_type  get_##x_in x_arg_list const                                                  \
      {                                                                                         \
        if (in_##x_in.ConnectionSize()==0)                                                      \
          {LERROR("in "<<ModuleUniqueCode()<<", in_" #x_in " must be connected."); lexit(df);}  \
        return in_##x_in.GetFirst x_param_list;                                                 \
      }

  GET_FROM_IN_PORT(action_set_size, const TInt&, (void), ())

  GET_FROM_IN_PORT(feature, const TFeature&, (void), ())

  GET_FROM_IN_PORT(reward, const TSingleReward&, (void), ())

  #undef GET_FROM_IN_PORT

  bool is_updatable() const
    {
      if (in_updatable.ConnectionSize()==0)  return true;
      return in_updatable.GetFirst();
    }

  TAction                 old_action;
  TAction                 next_action;
  TRealVector             old_policy;
  TRealVector             next_policy;
  TRealVector             oldQs;  //!< assigned in startAction(), kept during the action
  TRealVector             nextQs;
  TFeature                old_feature;
  TFeature                next_feature;
  TReal                   old_tau;
  TReal                   next_tau;

  TLSPIData  lspi_data_;

  TBool                   is_active_;
  TSingleReward           return_in_episode_;
  TInt                    actions_in_episode_;
  TBool                   is_end_of_episode_;
  TValue                  td_error_;
  TValue                  current_action_value_;

  inline TReal get_tau (void) const;
  inline TReal get_eps (void) const;
  // inline TReal get_alpha (void) const;

  //! tools
  int getFAUnitCount (void) const {return get_feature().length();}
  int getActionCount (void) const {return get_action_set_size();}

  void initializeW (void);

  //! sub execution methods
  TValue update_with_ql (const TValue &R);
  TValue update_with_sarsa (const TValue &R);

  //! select \p TAgentTDLFA::next_action from real actions and internal actions
  void        prepare_next_action (/*const TState &x*/);

  //! execution methods
  void        reset_episode (); //! execute this function to start new episode
  TAction     select_action ();
  bool        update (const TSingleReward &reward);

};
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
} // end of lspi_discaction_detail
//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_lspi_discaction_h
//-------------------------------------------------------------------------------------------
