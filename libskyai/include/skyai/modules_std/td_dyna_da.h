//-------------------------------------------------------------------------------------------
/*! \file    td_dyna_da.h
    \brief   libskyai - Dyna with TD(lambda) for discrete action space
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Nov.09, 2010

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

    \note Abbreviations:
      - PS : Prioritized Sweep
      - VI : Value Iteration
*/
//-------------------------------------------------------------------------------------------
#ifndef skyai_td_dyna_da_h
#define skyai_td_dyna_da_h
//-------------------------------------------------------------------------------------------
#include <skyai/modules_std/td_generic_fa.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------


struct TDynaScheme
{
  bool PostActionPS;   //!< execute prioritized sweep after each action
  bool PreEpisodeVI;   //!< execute value iteration before each episode
  bool PreEpisodePS;   //!< execute prioritized sweep before each episode
  bool FirstEpisodeVI;   //!< execute value iteration before first episode (omitted if PreEpisodeVI is true)
  bool FirstEpisodePS;   //!< execute prioritized sweep before first episode (omitted if PreEpisodePS is true)

  bool GatherPSSamples;  //!< gather samples for PS even if PS is not performed

  TDynaScheme(void) :
      PostActionPS     (true),
      PreEpisodeVI     (false),
      PreEpisodePS     (false),
      FirstEpisodeVI   (false),
      FirstEpisodePS   (false),
      GatherPSSamples  (false)
    {}

  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( PostActionPS     );
      ADD( PreEpisodeVI     );
      ADD( PreEpisodePS     );
      ADD( FirstEpisodeVI   );
      ADD( FirstEpisodePS   );
      ADD( GatherPSSamples  );
      #undef ADD
    }
};
namespace var_space{void Register (TDynaScheme &x, TVariableMap &mmap);}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
namespace tdgfa_tmpl_detail
{
//-------------------------------------------------------------------------------------------


//===========================================================================================
//!\brief Configurations of MTDDyna_TDiscreteAction
class TTDDynaConfigurations
//===========================================================================================
{
public:

  TBool        UsingDyna  ;
  TDynaScheme  DynaScheme ;

  TBool        DynaInitAVFByRewardModel;  //!< if true, Action Value Function is initialized with a reward model

  TReal        DynaPSFeatureValMin;     //!< (PS) if an element of feature vector is greater than this value, it is pushed into PQueue (i.e. used in prioritized sweep)
  TReal        DynaPSStateTrProbMin;    //!< (PS) if transition probability is less than this value, planning is not performed
  TInt         DynaPSPlanningCount;     //!< (PS) depth of planning
  TReal        DynaPSPlanningTDErrMin;  //!< (PS) if key (TD-Error) of PQueue is less than this value, it is not used for planning (valid if positive)
  TReal        DynaPSKeepUnplannedElem; //!< (PS) probability to keep (not pop) an element in PQueue when it is not used for planning
  TInt         DynaVINumIteration;      //!< (VI) iteration number of value iteration

  TReal        DynaPSAlpha;     //!< (PS) step-size parameter for planning (AlphaDecreasingFactor is shared)
  TReal        DynaPSAlphaMin;  //!< (PS) lower bound of alpha
  TReal        DynaVIAlpha;     //!< (VI) step-size parameter for planning (AlphaDecreasingFactor is shared)
  TReal        DynaVIAlphaMin;  //!< (VI) lower bound of alpha


  TTDDynaConfigurations (var_space::TVariableMap &mmap)
    :
      UsingDyna               (true),
      DynaInitAVFByRewardModel(true),
      DynaPSFeatureValMin     (0.2l),
      DynaPSStateTrProbMin    (0.3l),
      DynaPSPlanningCount     (5),
      DynaPSPlanningTDErrMin  (-1.0l),
      DynaPSKeepUnplannedElem (-1.0l),
      DynaVINumIteration      (5),
      DynaPSAlpha             (0.05l),
      DynaPSAlphaMin          (0.005l),
      DynaVIAlpha             (0.05l),
      DynaVIAlphaMin          (0.005l)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( UsingDyna      );
      ADD( DynaScheme     );

      ADD( DynaInitAVFByRewardModel  );

      ADD( DynaPSFeatureValMin       );
      ADD( DynaPSStateTrProbMin      );
      ADD( DynaPSPlanningCount       );
      ADD( DynaPSPlanningTDErrMin    );
      ADD( DynaPSKeepUnplannedElem   );
      ADD( DynaVINumIteration        );

      ADD( DynaPSAlpha        );
      ADD( DynaPSAlphaMin     );
      ADD( DynaVIAlpha        );
      ADD( DynaVIAlphaMin     );
      #undef ADD
    }

};
//-------------------------------------------------------------------------------------------



//===========================================================================================
/*!\brief Dyna with TD(lambda) for discrete action space
    \note dynamics model and reward model are NOT included in this module  */
class MTDDyna_TDiscreteAction
    : public MTDGenericFuncApprox<TDiscreteAction>
//===========================================================================================
{
public:
  typedef MTDGenericFuncApprox<TDiscreteAction>  TParent;
  typedef MTDDyna_TDiscreteAction                TThis;
  SKYAI_MODULE_NAMES(MTDDyna_TDiscreteAction)

  MTDDyna_TDiscreteAction (const std::string &v_instance_name)
    :
      TParent               (v_instance_name),
      dconf_                (TParent::param_box_config_map()),
      in_feature                       (*this),
      in_settable_avtable              (*this),
      in_dynm_trans_probability        (*this),
      in_dynm_next_feature             (*this),
      in_dynm_most_probable_action     (*this),
      in_rwdm_trans_reward             (*this),
      in_rwdm_trans_reward_at_feature  (*this)
    {
      add_in_port ( in_feature                       );
      add_in_port ( in_settable_avtable              );
      add_in_port ( in_dynm_trans_probability        );
      add_in_port ( in_dynm_next_feature             );
      add_in_port ( in_dynm_most_probable_action     );
      add_in_port ( in_rwdm_trans_reward             );
      add_in_port ( in_rwdm_trans_reward_at_feature  );
    }

  struct TDynaPQueueItem
    {
      typedef TDynaPQueueItem TThis;
      TInt     S;
      TReal    Key;
      TDynaPQueueItem(TInt v_s) : S(v_s), Key(0.0l) {}
      TDynaPQueueItem(TInt v_s, const TReal &v_k) : S(v_s), Key(v_k) {}
      bool operator==(const TThis &rhs) const {return S==rhs.S;}
      bool operator> (const TThis &rhs) const {return Key> rhs.Key;}
      bool operator>=(const TThis &rhs) const {return Key>=rhs.Key;}
      bool operator< (const TThis &rhs) const {return Key< rhs.Key;}
      bool operator<=(const TThis &rhs) const {return Key<=rhs.Key;}
    };

  TTDDynaConfigurations& DynaConfig()  {return dconf_;}
  const TTDDynaConfigurations& DynaConfig() const {return dconf_;}

  std::list<TDynaPQueueItem>&       PQueue()  {return pqueue_;}
  const std::list<TDynaPQueueItem>& PQueue() const {return pqueue_;}

  /*!\brief embed a prior knowledge to model; set reward source */
  void SetRewardSource(const std::list<TRealVector> &features, const std::list<TReal> &rewards);

protected:

  TTDDynaConfigurations  dconf_;

  std::list<TDynaPQueueItem> pqueue_;

  TRealVector next_phi_, old_phi_;

  TRealVector tmp_v_avnew_, tmp_v_phi_;


  //!\brief input a feature vector (output of basis functions) by this port
  MAKE_IN_PORT(in_feature, const TRealVector& (void), TThis);

  /*!\brief input settable parameter vector of a linear regressor; action value table
      (Theta[0,..,NA-1],  Theta[a][0,..,NK], NA: number of action, NK: number of basis functions) */
  MAKE_IN_PORT(in_settable_avtable, TRealVectorSet& (void), TThis);


  //!\brief (dynamics model) input estimated probability to transit from symbol state curr_s to next_s with action curr_a
  MAKE_IN_PORT(in_dynm_trans_probability, const TReal& (const TInt &curr_s, const TInt &next_s, const TDiscreteAction &curr_a), TThis);

  //!\brief (dynamics model) input next feature by taking action curr_a from current feature curr_phi
  MAKE_IN_PORT(in_dynm_next_feature, const TRealVector& (const TRealVector &curr_phi, const TDiscreteAction &curr_a), TThis);

  /*!\brief (dynamics model) input most probable action with which the current symbol state curr_s transits to next_s;
            its probability is stored into trans_prob */
  MAKE_IN_PORT(in_dynm_most_probable_action, const TDiscreteAction& (const TInt &curr_s, const TInt &next_s, TReal &trans_prob), TThis);


  //!\brief (reward model) input extimated reward by taking action curr_a at the current symbol state curr_s
  MAKE_IN_PORT(in_rwdm_trans_reward, const TReal& (const TInt &curr_s, const TDiscreteAction &curr_a), TThis);

  //!\brief (reward model) input extimated reward by taking action curr_a at the current feature curr_phi
  MAKE_IN_PORT(in_rwdm_trans_reward_at_feature, const TReal& (const TRealVector &curr_phi, const TDiscreteAction &curr_a), TThis);


  // generate get_##x_in (e.g. get_reward)
  #define GET_FROM_IN_PORT(x_in,x_return_type,x_arg_list,x_param_list)                          \
    x_return_type  get_##x_in x_arg_list const                                                  \
      {                                                                                         \
        if (in_##x_in.ConnectionSize()==0)                                                      \
          {LERROR("in "<<ModuleUniqueCode()<<", in_" #x_in " must be connected."); lexit(df);}  \
        return in_##x_in.GetFirst x_param_list;                                                 \
      }

  GET_FROM_IN_PORT(feature, const TRealVector&, (void), ())

  GET_FROM_IN_PORT(settable_avtable, TRealVectorSet&, (void), ())


  GET_FROM_IN_PORT(dynm_trans_probability, const TReal&, (const TInt &curr_s, const TInt &next_s, const TDiscreteAction &curr_a), (curr_s,next_s,curr_a))

  GET_FROM_IN_PORT(dynm_next_feature, const TRealVector&, (const TRealVector &curr_phi, const TDiscreteAction &curr_a), (curr_phi,curr_a))

  GET_FROM_IN_PORT(dynm_most_probable_action, const TDiscreteAction&, (const TInt &curr_s, const TInt &next_s, TReal &trans_prob), (curr_s,next_s,trans_prob))

  GET_FROM_IN_PORT(rwdm_trans_reward, const TReal&, (const TInt &curr_s, const TDiscreteAction &curr_a), (curr_s,curr_a))

  GET_FROM_IN_PORT(rwdm_trans_reward_at_feature, const TReal&, (const TRealVector &curr_phi, const TDiscreteAction &curr_a), (curr_phi,curr_a))

  #undef GET_FROM_IN_PORT


  override void slot_initialize_exec (void);

  override void prepare_next_action()
    {
      next_phi_= get_feature();
      TParent::prepare_next_action();
    }

  override TDiscreteAction select_action()
    {
      old_phi_= next_phi_;
      return TParent::select_action();
    }

  override void reset_episode();

  override void update(const TSingleReward &reward);

  void insert_to_pqueue(TInt s, const TReal &tderror);
  void dyna_ps_common (const TReal &reward, const TReal &tderror);
  void dyna_value_iteration (void);
  void dyna_prioritized_sweep (void);
  TReal dyna_update_by_model (TInt v_old_state, TDiscreteAction v_action, const TReal &alpha, TInt NA, TInt NS);

  inline TReal get_ps_alpha (void) const;

  inline TReal get_vi_alpha (void) const;

};
//-------------------------------------------------------------------------------------------

inline TReal MTDDyna_TDiscreteAction::get_ps_alpha (void) const
{
  TReal alpha (0.0l);
  alpha= dconf_.DynaPSAlpha * real_exp(-conf_.AlphaDecreasingFactor * static_cast<TReal>(mem_.EpisodeNumber));
  return ApplyRange(alpha,dconf_.DynaPSAlphaMin,dconf_.DynaPSAlpha);
}
inline TReal MTDDyna_TDiscreteAction::get_vi_alpha (void) const
{
  TReal alpha (0.0l);
  alpha= dconf_.DynaVIAlpha * real_exp(-conf_.AlphaDecreasingFactor * static_cast<TReal>(mem_.EpisodeNumber));
  return ApplyRange(alpha,dconf_.DynaVIAlphaMin,dconf_.DynaVIAlpha);
}
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
} // end of tdgfa_tmpl_detail
//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_td_dyna_da_h
//-------------------------------------------------------------------------------------------
