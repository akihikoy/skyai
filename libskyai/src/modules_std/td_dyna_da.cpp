//-------------------------------------------------------------------------------------------
/*! \file    td_dyna_da.cpp
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
*/
//-------------------------------------------------------------------------------------------
#include <skyai/modules_std/td_dyna_da.h>
#include <lora/type_gen_oct.h>
#include <lora/stl_math.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{


namespace var_space
{
  void Register(TDynaScheme &x, TVariableMap &mmap)  {x.Register(mmap);}
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
namespace tdgfa_tmpl_detail
{
using namespace std;
// using namespace boost;


//===========================================================================================
// class MTDDyna_TDiscreteAction
//===========================================================================================

/*!\brief embed a prior knowledge to model; set reward source */
void MTDDyna_TDiscreteAction::SetRewardSource(const std::list<TRealVector> &features, const std::list<TReal> &rewards)
{
  LASSERT1op1(features.size(),==,rewards.size());

  std::list<TReal>::const_iterator r_itr(rewards.begin());
  for (std::list<TRealVector>::const_iterator f_itr(features.begin()),f_last(features.end()); f_itr!=f_last; ++f_itr)
  {
    insert_to_pqueue(max_element_index(GenBegin(*f_itr),GenEnd(*f_itr)), *r_itr);
  }
}
//-------------------------------------------------------------------------------------------

override void MTDDyna_TDiscreteAction::slot_initialize_exec (void)
{
  TParent::slot_initialize_exec();

  if (dconf_.UsingDyna
      && mem_.EpisodeNumber==0
      && dconf_.DynaInitAVFByRewardModel
      && in_settable_avtable.ConnectionSize()!=0
      && in_rwdm_trans_reward.ConnectionSize()!=0)
  {
    TRealVectorSet &avtable(get_settable_avtable());
    const TInt NA(GenSize(avtable));    LASSERT1op1(NA,>,0);
    const TInt NS(GenSize(avtable[0])); LASSERT1op1(NS,>,0);

    for (int a(0); a<NA; ++a)
      for (int s(0); s<NS; ++s)
        GenAt(avtable[a],s)= get_rwdm_trans_reward(s,a);
  }
}
//-------------------------------------------------------------------------------------------

override void MTDDyna_TDiscreteAction::reset_episode()
{
  TParent::reset_episode();

  if (dconf_.UsingDyna && is_updatable())
  {
    if (dconf_.DynaScheme.PreEpisodeVI || (mem_.EpisodeNumber==0 && dconf_.DynaScheme.FirstEpisodeVI))
      dyna_value_iteration();

    if (dconf_.DynaScheme.PreEpisodePS || (mem_.EpisodeNumber==0 && dconf_.DynaScheme.FirstEpisodePS))
      dyna_prioritized_sweep();
  }
}
//-------------------------------------------------------------------------------------------

override void MTDDyna_TDiscreteAction::update(const TSingleReward &reward)
{
  TParent::update(reward);

  if (dconf_.UsingDyna && is_updatable())
    dyna_ps_common (reward, td_error_);
}
//-------------------------------------------------------------------------------------------

void MTDDyna_TDiscreteAction::insert_to_pqueue(TInt s, const TReal &tderror)
{
  TDynaPQueueItem item(s);
  std::list<TDynaPQueueItem>::iterator itr;
  if((itr=std::find(pqueue_.begin(),pqueue_.end(),item))!=pqueue_.end())
    {item=*itr; pqueue_.erase(itr);}
  item.Key=tderror;
  pqueue_.insert(find_if(pqueue_.begin(),pqueue_.end(),std::bind2nd(std::less<TDynaPQueueItem>(),item)),item);
}
//-------------------------------------------------------------------------------------------

void MTDDyna_TDiscreteAction::dyna_ps_common (const TReal &reward, const TReal &tderror)
{
  if (dconf_.DynaScheme.PostActionPS || dconf_.DynaScheme.PreEpisodePS || dconf_.DynaScheme.GatherPSSamples)
  {
    /* insert into Priority Queue (PQueue) */
    for(TInt s(0),NS(GenSize(old_phi_)); s<NS; ++s)
      if(real_fabs(GenAt(old_phi_,s)) > dconf_.DynaPSFeatureValMin)
        insert_to_pqueue(s,real_fabs(tderror*GenAt(old_phi_,s)));
  }

// LDEBUG(pqueue_.size()<<"  "<<(pqueue_.empty()?-1:pqueue_.front().S)<<"  "<<(pqueue_.empty()?0.0l:pqueue_.front().Key));

  if (dconf_.DynaScheme.PostActionPS)
  {
    /* planning part */
    dyna_prioritized_sweep();

    // prepareNextAction, again.
    if(!is_end_of_episode_ && dconf_.DynaPSPlanningCount>0)  prepare_next_action();
  }
}
//-------------------------------------------------------------------------------------------


void MTDDyna_TDiscreteAction::dyna_value_iteration (void)
{
  const TRealVectorSet &avtable(get_settable_avtable());
  const TInt NA(GenSize(avtable));    LASSERT1op1(NA,>,0);
  const TInt NS(GenSize(avtable[0])); LASSERT1op1(NS,>,0);
  const TReal alpha(get_vi_alpha());
  for(int i(0); i<dconf_.DynaVINumIteration; ++i)
  {
    for(TInt v_old_state(0); v_old_state<NS; ++v_old_state)
    {
      for(TDiscreteAction v_action(0); v_action<NA; ++v_action)
      {
        /*TReal v_tderror=*/ dyna_update_by_model (v_old_state, v_action, alpha, NA, NS);
      }
    }
  }
}
//-------------------------------------------------------------------------------------------

/*FIXME*/
// #define _PS_VER_2

#if defined _PS_VER_2
static TReal get_max_av(const TRealVectorSet &avtable, TInt state)
{
  TReal avmax(-REAL_MAX),avnow;
  for(TRealVectorSet::const_iterator itr(avtable.begin()),last(avtable.end()); itr!=last; ++itr)
    if((avnow=GenAt(*itr,state))>avmax)  avmax=avnow;
  return avmax;
}
#endif
//-------------------------------------------------------------------------------------------

void MTDDyna_TDiscreteAction::dyna_prioritized_sweep (void)
{
  const TRealVectorSet &avtable(get_settable_avtable());
  const TInt NA(GenSize(avtable));    LASSERT1op1(NA,>,0);
  const TInt NS(GenSize(avtable[0])); LASSERT1op1(NS,>,0);

  const TReal alpha(get_ps_alpha());
  for(int p(0); p<dconf_.DynaPSPlanningCount&&(!pqueue_.empty()); ++p)
  {
    TInt v_state(pqueue_.front().S);
    if (dconf_.DynaPSPlanningTDErrMin>0.0 && pqueue_.front().Key < dconf_.DynaPSPlanningTDErrMin)  {pqueue_.pop_front(); break;}

    bool planning_is_performed (false);
    // if dconf_.DynaPSKeepUnplannedElem <= 0, pqueue_ is popped,
    // else if planning is not performed in the following loop, pqueue_ is popped with probability (1-dconf_.DynaPSKeepUnplannedElem)
    if (dconf_.DynaPSKeepUnplannedElem<=DBL_TINY)
      pqueue_.pop_front();

    #if defined _PS_VER_2
    TReal  V_v_state= get_max_av(avtable,v_state);
    #endif
    for(TInt v_old_state(0); v_old_state<NS; ++v_old_state)
    {
      /* planning is not performed for action with which the state transits
          to the same state (for stability of planning):  */
      if (v_state==v_old_state)  continue;

      #if defined _PS_VER_2
      /* planning is not performed if the state value of the previous state is greater than that of the current state
          (for fast propagation): */
      if (get_max_av(avtable,v_old_state) > V_v_state)
        continue;
      #endif

      TReal trans_prob(0.0);
      TDiscreteAction v_action(-1);
      v_action= get_dynm_most_probable_action (v_old_state, v_state, trans_prob);

      if(v_action==-1 || trans_prob<dconf_.DynaPSStateTrProbMin) continue;

      if (dconf_.DynaPSKeepUnplannedElem>DBL_TINY && !planning_is_performed)
      {
        pqueue_.pop_front();
        planning_is_performed= true;
      }

      // local planning: v_old_state --v_action--> v_state
      TReal v_tderror=  dyna_update_by_model (v_old_state, v_action, alpha, NA, NS);

      insert_to_pqueue(v_old_state, real_fabs(v_tderror*trans_prob));

    }
    if (dconf_.DynaPSKeepUnplannedElem>DBL_TINY && !planning_is_performed)
    {
      if (RollADice(dconf_.DynaPSKeepUnplannedElem))
        break;  // : planning is not performed and pqueue_ is not popped.
                //   in this case, continuing planning repeats the same process, which is nonsense. thus, the planning is terminated
      else
        pqueue_.pop_front();
    }
  }
LDEBUG(pqueue_.size()<<"  "<<(pqueue_.empty()?-1:pqueue_.front().S)<<"  "<<(pqueue_.empty()?0.0l:pqueue_.front().Key));
}
//-------------------------------------------------------------------------------------------

//! \brief (dyna) update Q by value iteration using the learnt model
TReal MTDDyna_TDiscreteAction::dyna_update_by_model (TInt v_old_state, TDiscreteAction v_action, const TReal &alpha, TInt NA, TInt NS)
{
  TRealVectorSet &avtable(get_settable_avtable());

  /*calculate tmp_v_avnew_*/{
    GenResize(tmp_v_avnew_,NA);
    std::fill (GenBegin(tmp_v_avnew_),GenEnd(tmp_v_avnew_), 0.0l);
    GenResize(tmp_v_phi_,NS);
    std::fill (GenBegin(tmp_v_phi_),GenEnd(tmp_v_phi_), 0.0l);

    int s(0);
    for(TypeExt<TRealVector>::iterator vp_itr(GenBegin(tmp_v_phi_)),vp_last(GenEnd(tmp_v_phi_)); vp_itr!=vp_last; ++vp_itr,++s)
      *vp_itr = get_dynm_trans_probability(v_old_state,s,v_action);

    TRealVectorSet::const_iterator qitr(avtable.begin());
    for(TypeExt<TRealVector>::iterator vav_itr(GenBegin(tmp_v_avnew_)),vav_last(GenEnd(tmp_v_avnew_)); vav_itr!=vav_last; ++vav_itr,++qitr)
      *vav_itr = InnerProd(GenBegin(*qitr),GenEnd(*qitr), GenBegin(tmp_v_phi_));
  }
  TReal V_v_new = *std::max_element(GenBegin(tmp_v_avnew_),GenEnd(tmp_v_avnew_));
  TypeExt<TRealVector>::value_type &qt(GenAt(avtable[v_action],v_old_state));
  TReal v_tderror = get_rwdm_trans_reward(v_old_state,v_action) + conf_.Gamma * V_v_new - qt;
  qt = qt + alpha * v_tderror;
  return v_tderror;
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
SKYAI_ADD_MODULE(MTDDyna_TDiscreteAction)
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace tdgfa_tmpl_detail
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

