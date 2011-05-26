//-------------------------------------------------------------------------------------------
/*! \file    models.cpp
    \brief   libskyai - dynamics and reward models
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Nov.09, 2010-

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
#include <skyai/modules_std/models.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
using namespace std;
// using namespace boost;


namespace var_space
{
  void Register(TSimpleDMParamAct &x, TVariableMap &mmap)  {x.Register(mmap);}
  void Register(TMixFS2DMParamAct &x, TVariableMap &mmap)  {x.Register(mmap);}
  void Register(TMixFS3DMParamAct &x, TVariableMap &mmap)  {x.Register(mmap);}
  void Register(TSimpleRMParamAct &x, TVariableMap &mmap)  {x.Register(mmap);}
}
//-------------------------------------------------------------------------------------------


//!\brief constrain a state transition probability matrix
static void constrain_transition_prob_matrix (TRealMatrix &Fa)
{
  /*FIXME DBG*/
  const int  cols(Fa.cols()), rows(Fa.rows());
  for(int s1(0); s1<cols; ++s1)
  {
    double sum(0.0);
    for(int s2(0); s2<rows; ++s2)
    {
      //*dbg*/if(Fa(s2,s1)>1.1)std::cerr<<"Fa("<<s2<<","<<s1<<")="<<Fa(s2,s1)<<endl;
      Fa(s2,s1)= ApplyRange(Fa(s2,s1),0.0,1.0);
      sum+=Fa(s2,s1);
      //*improper*/sum+=_square(Fa(s2,s1));
    }
    //*dbg*/if(sum>1.0) for(TUnitIndex s2(0); s2<fasize; ++s2) Fa(s2,s1)/=sum;
    //*improper*/if((sum=real_sqrt(sum))>1.0) for(TUnitIndex s2(0); s2<fasize; ++s2) Fa(s2,s1)/=sum;
    if(sum>1.0) for(int s2(0); s2<rows; ++s2) Fa(s2,s1)/=sum;
  }
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class MSimpleDynamicsModel
//===========================================================================================

void MSimpleDynamicsModel::setup_param() const
{
  TInt  action_size(get_action_set_size());
  TInt  feature_size(GenSize(get_feature()));
  if (action_size*feature_size==0)
  {
    LWARNING(InstanceName()<<"::setup_param: size of mem_.Param is zero;"
              <<" action_size="<<action_size<<", feature_size="<<feature_size);
    mem_.Param.clear();
    return;
  }
  if (static_cast<TInt>(mem_.Param.size())!=action_size)
  {
    mem_.Param.resize(action_size, TSimpleDMParamAct(feature_size));
  }
  if (mem_.Param[0].Fa.cols()!=feature_size)
  {
    for (std::vector<TSimpleDMParamAct>::iterator p_itr(mem_.Param.begin()),p_last(mem_.Param.end()); p_itr!=p_last; ++p_itr)
      p_itr->Fa.resize(feature_size,feature_size,0.0l);
  }
}
//-------------------------------------------------------------------------------------------

override const TReal& MSimpleDynamicsModel::out_trans_probability_get (const TInt &curr_s, const TInt &next_s, const TDiscreteAction &curr_a) const
{
  setup_param();
  tmp_trans_prob_= mem_.Param[curr_a].Fa(next_s,curr_s);
  return tmp_trans_prob_;
}
//-------------------------------------------------------------------------------------------

override const TRealVector& MSimpleDynamicsModel::out_next_feature_get (const TRealVector &curr_phi, const TDiscreteAction &curr_a) const
{
  setup_param();
  tmp_next_phi_= mem_.Param[curr_a].Fa * curr_phi;
  return tmp_next_phi_;
}
//-------------------------------------------------------------------------------------------

override const TDiscreteAction& MSimpleDynamicsModel::out_most_probable_action_get (const TInt &curr_s, const TInt &next_s, TReal &trans_prob) const
{
  setup_param();
  trans_prob= 0.0l;
  tmp_trans_a_= -1;
  for (TDiscreteAction a(0),end_a(mem_.Param.size()); a<end_a; ++a)
  {
    if (mem_.Param[a].Fa(next_s,curr_s) > trans_prob)
    {
      trans_prob= mem_.Param[a].Fa(next_s,curr_s);
      tmp_trans_a_= a;
    }
  }
  return tmp_trans_a_;
}
//-------------------------------------------------------------------------------------------

override void MSimpleDynamicsModel::slot_initialize_exec (void)
{
  setup_param();
}
//-------------------------------------------------------------------------------------------

override void MSimpleDynamicsModel::slot_start_action_exec (const TDiscreteAction &curr_a)
{
  setup_param();
  tmp_old_phi_ = get_feature();
  tmp_old_a_   = curr_a;
}
//-------------------------------------------------------------------------------------------

override void MSimpleDynamicsModel::slot_finish_action_exec (void)
{
  setup_param();
  const TRealVector &next_phi(get_feature()), &phi(tmp_old_phi_);
  TRealMatrix  &Fa(mem_.Param[tmp_old_a_].Fa);
  Fa+= get_alpha()*(next_phi-Fa*phi)*phi.transpose();
  constrain_transition_prob_matrix (Fa);
}
//-------------------------------------------------------------------------------------------

inline TReal MSimpleDynamicsModel::get_alpha (void) const
{
  TReal alpha (0.0l);
  alpha= conf_.Alpha * real_exp(-conf_.AlphaDecreasingFactor * static_cast<TReal>(mem_.EpisodeNumber));
  return ApplyRange(alpha,conf_.AlphaMin,conf_.Alpha);
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class MMixFS2DynamicsModel
//===========================================================================================

void MMixFS2DynamicsModel::setup_param() const
{
  TInt  action_size(get_action_set_size());
  TInt  feature_size(GenSize(get_feature()));
  TInt  state_dim(GenSize(get_state()));
  if (action_size*feature_size*state_dim==0)
  {
    LWARNING(InstanceName()<<"::setup_param: size of mem_.Param is zero;"
              <<" action_size="<<action_size<<", feature_size="<<feature_size<<", state_dim="<<state_dim);
    mem_.Param.clear();
    return;
  }
  if (static_cast<TInt>(mem_.Param.size())!=action_size)
  {
    mem_.Param.resize(action_size, TMixFS2DMParamAct(feature_size,state_dim));
  }
  if (mem_.Param[0].Fa.cols()!=feature_size || GenSize(mem_.Param[0].Dxa)!=state_dim)
  {
    for (std::vector<TMixFS2DMParamAct>::iterator p_itr(mem_.Param.begin()),p_last(mem_.Param.end()); p_itr!=p_last; ++p_itr)
      p_itr->Resize(feature_size,state_dim);
  }
}
//-------------------------------------------------------------------------------------------

void MMixFS2DynamicsModel::update_cache (const TDiscreteAction &a) const
{
  TRealMatrix        &CachedFa(mem_.Param[a].CachedFa);
  const TRealMatrix  &Fa(mem_.Param[a].Fa),  &Wa(mem_.Param[a].Wa);
  const TRealVector  &Dxa(mem_.Param[a].Dxa);
  TInt  feature_size(GenSize(get_feature()));

  CachedFa.resize (feature_size,feature_size,0.0l);

  for (int c(0),cols(CachedFa.cols()); c<cols; ++c)
  {
    TRealVector phix;
    get_state_to_feature (get_center_state_set()[c] + Wa.column(c) + Dxa, phix);
    for (int r(0),rows(CachedFa.rows()); r<rows; ++r)
      CachedFa(r,c)= Fa(r,c) + phix(r);
  }
  constrain_transition_prob_matrix (CachedFa);

  mem_.Param[a].IsCached= true;
}
//-------------------------------------------------------------------------------------------

override const TReal& MMixFS2DynamicsModel::out_trans_probability_get (const TInt &curr_s, const TInt &next_s, const TDiscreteAction &curr_a) const
{
  setup_param();
  if (!mem_.Param[curr_a].IsCached)  update_cache(curr_a);
  tmp_trans_prob_= mem_.Param[curr_a].CachedFa(next_s,curr_s);
  return tmp_trans_prob_;
}
//-------------------------------------------------------------------------------------------

override const TRealVector& MMixFS2DynamicsModel::out_next_feature_get (const TRealVector &curr_phi, const TDiscreteAction &curr_a) const
{
  setup_param();
  if (!mem_.Param[curr_a].IsCached)  update_cache(curr_a);
  tmp_next_phi_= mem_.Param[curr_a].CachedFa * curr_phi;
  return tmp_next_phi_;
}
//-------------------------------------------------------------------------------------------

override const TDiscreteAction& MMixFS2DynamicsModel::out_most_probable_action_get (const TInt &curr_s, const TInt &next_s, TReal &trans_prob) const
{
  setup_param();
  trans_prob= 0.0l;
  tmp_trans_a_= -1;
  for (TDiscreteAction a(0),end_a(mem_.Param.size()); a<end_a; ++a)
  {
    if (!mem_.Param[a].IsCached)  update_cache(a);
    if (mem_.Param[a].CachedFa(next_s,curr_s) > trans_prob)
    {
      trans_prob= mem_.Param[a].CachedFa(next_s,curr_s);
      tmp_trans_a_= a;
    }
  }
  return tmp_trans_a_;
}
//-------------------------------------------------------------------------------------------

override void MMixFS2DynamicsModel::slot_initialize_exec (void)
{
  setup_param();
}
//-------------------------------------------------------------------------------------------

override void MMixFS2DynamicsModel::slot_start_action_exec (const TDiscreteAction &curr_a)
{
  setup_param();
  tmp_old_phi_ = get_feature();
  tmp_old_x_   = get_state();
  tmp_old_a_   = curr_a;
}
//-------------------------------------------------------------------------------------------

override void MMixFS2DynamicsModel::slot_finish_action_exec (void)
{
  setup_param();
  mem_.Param[tmp_old_a_].IsCached= false;
  const TReal alpha(get_alpha());
  const TRealVector &next_phi(get_feature()), &phi(tmp_old_phi_);
  const TRealVector &next_x(get_state()), &x(tmp_old_x_);
  TRealMatrix  &Fa(mem_.Param[tmp_old_a_].Fa), &Wa(mem_.Param[tmp_old_a_].Wa);
  TRealVector  &Dxa(mem_.Param[tmp_old_a_].Dxa);

  TRealVector  est_next_x= x + Wa*phi + Dxa;
  TRealVector  tmp_phi;  get_state_to_feature(est_next_x, tmp_phi);
  TRealVector  ext_next_phi= Fa*phi + tmp_phi;
  Wa+= alpha*(next_x-est_next_x)*phi.transpose();
  Dxa+= alpha*(next_x-est_next_x);
  Fa+= alpha*(next_phi-ext_next_phi)*phi.transpose();

  /*FIXME dbg*-/{
    const int  fasize (feature_func->size());
    for(TInt s(0); s<fasize; ++s)
    {
      double sum(0.0);
      for(TInt s2(0); s2<fasize; ++s2)
      {
        Fa(s2,s)=ApplyRange(Fa(s2,s),0.0,1.0);
        sum+=Fa(s2,s);
      }
      if(sum>1.0) for(TInt s2(0); s2<getFAUnitCount(); ++s2) Fa(s2,s)/=sum;
    }
  }//*/
}
//-------------------------------------------------------------------------------------------

inline TReal MMixFS2DynamicsModel::get_alpha (void) const
{
  TReal alpha (0.0l);
  alpha= conf_.Alpha * real_exp(-conf_.AlphaDecreasingFactor * static_cast<TReal>(mem_.EpisodeNumber));
  return ApplyRange(alpha,conf_.AlphaMin,conf_.Alpha);
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class MMixFS3DynamicsModel
//===========================================================================================

void MMixFS3DynamicsModel::setup_param() const
{
  TInt  action_size(get_action_set_size());
  TInt  feature_size(GenSize(get_feature()));
  TInt  state_dim(GenSize(get_state()));
  if (action_size*feature_size*state_dim==0)
  {
    LWARNING(InstanceName()<<"::setup_param: size of mem_.Param is zero;"
              <<" action_size="<<action_size<<", feature_size="<<feature_size<<", state_dim="<<state_dim);
    mem_.Param.clear();
    return;
  }
  if (static_cast<TInt>(mem_.Param.size())!=action_size)
  {
    mem_.Param.resize(action_size, TMixFS3DMParamAct(feature_size,state_dim));
  }
  if (mem_.Param[0].Fa.cols()!=feature_size || GenSize(mem_.Param[0].Dxa)!=state_dim)
  {
    for (std::vector<TMixFS3DMParamAct>::iterator p_itr(mem_.Param.begin()),p_last(mem_.Param.end()); p_itr!=p_last; ++p_itr)
      p_itr->Resize(feature_size,state_dim);
  }
}
//-------------------------------------------------------------------------------------------

void MMixFS3DynamicsModel::update_cache (const TDiscreteAction &a) const
{
  TRealMatrix        &CachedFa(mem_.Param[a].CachedFa);
  const TRealMatrix  &Fa(mem_.Param[a].Fa),  &Aa(mem_.Param[a].Aa),  &Ba(mem_.Param[a].Ba);
  const TRealVector  &Dxa(mem_.Param[a].Dxa);
  TInt  feature_size(GenSize(get_feature()));

  CachedFa.resize (feature_size,feature_size,0.0l);

  for (int c(0),cols(CachedFa.cols()); c<cols; ++c)
  {
    const TRealVector &x_mu(get_center_state_set()[c]);
    TRealVector phix;
    get_state_to_feature (x_mu + Aa*x_mu + Ba.column(c) + Dxa, phix);
    for (int r(0),rows(CachedFa.rows()); r<rows; ++r)
      CachedFa(r,c)= Fa(r,c) + phix(r);
  }
  constrain_transition_prob_matrix (CachedFa);

  mem_.Param[a].IsCached= true;
}
//-------------------------------------------------------------------------------------------

override const TReal& MMixFS3DynamicsModel::out_trans_probability_get (const TInt &curr_s, const TInt &next_s, const TDiscreteAction &curr_a) const
{
  setup_param();
  if (!mem_.Param[curr_a].IsCached)  update_cache(curr_a);
  tmp_trans_prob_= mem_.Param[curr_a].CachedFa(next_s,curr_s);
  return tmp_trans_prob_;
}
//-------------------------------------------------------------------------------------------

override const TRealVector& MMixFS3DynamicsModel::out_next_feature_get (const TRealVector &curr_phi, const TDiscreteAction &curr_a) const
{
  setup_param();
  if (!mem_.Param[curr_a].IsCached)  update_cache(curr_a);
  tmp_next_phi_= mem_.Param[curr_a].CachedFa * curr_phi;
  return tmp_next_phi_;
}
//-------------------------------------------------------------------------------------------

override const TDiscreteAction& MMixFS3DynamicsModel::out_most_probable_action_get (const TInt &curr_s, const TInt &next_s, TReal &trans_prob) const
{
  setup_param();
  trans_prob= 0.0l;
  tmp_trans_a_= -1;
  for (TDiscreteAction a(0),end_a(mem_.Param.size()); a<end_a; ++a)
  {
    if (!mem_.Param[a].IsCached)  update_cache(a);
    if (mem_.Param[a].CachedFa(next_s,curr_s) > trans_prob)
    {
      trans_prob= mem_.Param[a].CachedFa(next_s,curr_s);
      tmp_trans_a_= a;
    }
  }
  return tmp_trans_a_;
}
//-------------------------------------------------------------------------------------------

override void MMixFS3DynamicsModel::slot_initialize_exec (void)
{
  setup_param();
}
//-------------------------------------------------------------------------------------------

override void MMixFS3DynamicsModel::slot_start_action_exec (const TDiscreteAction &curr_a)
{
  setup_param();
  tmp_old_phi_ = get_feature();
  tmp_old_x_   = get_state();
  tmp_old_a_   = curr_a;
}
//-------------------------------------------------------------------------------------------

override void MMixFS3DynamicsModel::slot_finish_action_exec (void)
{
  setup_param();
  mem_.Param[tmp_old_a_].IsCached= false;
  const TReal alpha(get_alpha());
  const TRealVector &next_phi(get_feature()), &phi(tmp_old_phi_);
  const TRealVector &next_x(get_state()), &x(tmp_old_x_);
  TRealMatrix  &Fa(mem_.Param[tmp_old_a_].Fa), &Aa(mem_.Param[tmp_old_a_].Aa), &Ba(mem_.Param[tmp_old_a_].Ba);
  TRealVector  &Dxa(mem_.Param[tmp_old_a_].Dxa);

  TRealVector  est_next_x= x + Aa*x + Ba*phi + Dxa;
  TRealVector  tmp_phi;  get_state_to_feature(est_next_x, tmp_phi);
  TRealVector ext_next_phi= Fa*phi + tmp_phi;
  Aa+= mconf_.AaAlphaRate*alpha*(next_x-est_next_x)*x.transpose();
  Ba+= alpha*(next_x-est_next_x)*phi.transpose();
  Dxa+= alpha*(next_x-est_next_x);
  Fa+= alpha*(next_phi-ext_next_phi)*phi.transpose();
}
//-------------------------------------------------------------------------------------------

inline TReal MMixFS3DynamicsModel::get_alpha (void) const
{
  TReal alpha (0.0l);
  alpha= conf_.Alpha * real_exp(-conf_.AlphaDecreasingFactor * static_cast<TReal>(mem_.EpisodeNumber));
  return ApplyRange(alpha,conf_.AlphaMin,conf_.Alpha);
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class MSimpleRewardModel
//===========================================================================================

void MSimpleRewardModel::setup_param() const
{
  TInt  action_size(get_action_set_size());
  TInt  feature_size(GenSize(get_feature()));
  if (action_size*feature_size==0)
  {
    LWARNING(InstanceName()<<"::setup_param: size of mem_.Param is zero;"
              <<" action_size="<<action_size<<", feature_size="<<feature_size);
    mem_.Param.clear();
    return;
  }
  if (static_cast<TInt>(mem_.Param.size())!=action_size)
  {
    mem_.Param.resize(action_size, TSimpleRMParamAct(feature_size));
  }
  if (GenSize(mem_.Param[0].Ba)!=feature_size)
  {
    for (std::vector<TSimpleRMParamAct>::iterator p_itr(mem_.Param.begin()),p_last(mem_.Param.end()); p_itr!=p_last; ++p_itr)
      p_itr->Resize(feature_size);
  }
}
//-------------------------------------------------------------------------------------------

/*!\brief embed a prior knowledge to model; set reward source */
override void MSimpleRewardModel::SetRewardSource (const std::list<TRealVector> &features, const std::list<TReal> &rewards)
{
  LASSERT1op1(features.size(),==,rewards.size());

  setup_param();
  if (mem_.Param.size()==0 || GenSize(mem_.Param[0].Ba)==0)  return;

  //FIXME: lazy code (large comp. cost)
  TRealMatrix P;
  TRealVector R(rewards.size());
  for (std::list<TRealVector>::const_iterator itr(features.begin()); itr!=features.end(); ++itr)
  {
    if (P.rows()==0)  P= TRealMatrix(itr->transpose());
    else              P= P.stack(itr->transpose());
  }
  int i(0);
  for (std::list<TReal>::const_iterator itr(rewards.begin()); itr!=rewards.end(); ++itr,++i)
    GenAt(R,i)= *itr;

  TRealVector Ba_cmn= P.pseudo_inverse()*R;
// LDBGVAR(Ba_cmn.transpose());
  // assign a common reward model param Ba_cmn to each mem_.Param[action]
  for (std::vector<TSimpleRMParamAct>::iterator p_itr(mem_.Param.begin()),p_last(mem_.Param.end()); p_itr!=p_last; ++p_itr)
    p_itr->Ba= Ba_cmn;
// LDBGVAR(InstanceName());
// LDBGVAR(mem_.Param[0].Ba.transpose());
}
//-------------------------------------------------------------------------------------------

override const TReal& MSimpleRewardModel::out_trans_reward_get (const TInt &curr_s, const TDiscreteAction &curr_a) const
{
  setup_param();
  tmp_r_= mem_.Param[curr_a].Ba(curr_s);
  return tmp_r_;
}
//-------------------------------------------------------------------------------------------

override const TReal& MSimpleRewardModel::out_trans_reward_at_feature_get (const TRealVector &curr_phi, const TDiscreteAction &curr_a) const
{
  setup_param();
  LASSERT1op1(GenSize(curr_phi),==,GenSize(mem_.Param[curr_a].Ba));
  tmp_r_=  InnerProd(GenBegin(curr_phi),GenEnd(curr_phi),GenBegin(mem_.Param[curr_a].Ba));
  return tmp_r_;
}
//-------------------------------------------------------------------------------------------

override void MSimpleRewardModel::slot_initialize_exec (void)
{
  setup_param();
}
//-------------------------------------------------------------------------------------------

override void MSimpleRewardModel::slot_start_action_exec (const TDiscreteAction &curr_a)
{
  setup_param();
  tmp_old_phi_ = get_feature();
  tmp_old_a_   = curr_a;
}
//-------------------------------------------------------------------------------------------

override void MSimpleRewardModel::slot_finish_action_exec (void)
{
  setup_param();
  const TSingleReward curr_r(get_reward());

  const TRealVector &phi(tmp_old_phi_);
  TRealVector &Ba(mem_.Param[tmp_old_a_].Ba);
  TReal r_err= curr_r - InnerProd(GenBegin(phi),GenEnd(phi),GenBegin(Ba));
  Ba+= get_alpha()*r_err*phi;
}
//-------------------------------------------------------------------------------------------

inline TReal MSimpleRewardModel::get_alpha (void) const
{
  TReal alpha (0.0l);
  alpha= conf_.Alpha * real_exp(-conf_.AlphaDecreasingFactor * static_cast<TReal>(mem_.EpisodeNumber));
  return ApplyRange(alpha,conf_.AlphaMin,conf_.Alpha);
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
SKYAI_ADD_MODULE(MSimpleDynamicsModel)
SKYAI_ADD_MODULE(MMixFS2DynamicsModel)
SKYAI_ADD_MODULE(MMixFS3DynamicsModel)
SKYAI_ADD_MODULE(MSimpleRewardModel)
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

