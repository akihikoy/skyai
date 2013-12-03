//-------------------------------------------------------------------------------------------
/*! \file    avf_linear_discaction_b.cpp
    \brief   libskyai - batch-update version of linear function approximator module for action value function over discrete action space (source)
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
#include <skyai/modules_std/avf_linear_discaction_b.h>
#include <lora/stl_ext.h>
#include <lora/stl_math.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
namespace avf_linear_discaction_b_detail
{
using namespace std;
// using namespace boost;


/*!\brief calculate action value at the feature phi with action value parameter table Q, and store in resQs */
static void calc_action_value (const std::vector<TRealVector> &Q, TRealVector &resQs, const TRealVector &phi)
{
  double *qs_itr(GenBegin(resQs));
  for (std::vector<TRealVector>::const_iterator qitr(Q.begin()); qitr!=Q.end(); ++qitr, ++qs_itr)
    *qs_itr= InnerProd(GenBegin(*qitr),GenEnd(*qitr),OctBegin(phi));
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class MAVFLinearDiscActionB
//===========================================================================================

// NOTE: the same as MAVFLinearDiscAction (FIXME: define a base class)
override void MAVFLinearDiscActionB::slot_initialize_exec (void)
{
  if (param_.BFSize()*param_.ActionSetSize()==0)
  {
    // parameter initialization...

    param_.Init (get_feature_dim(), get_action_set_size());

    if (param_.BFSize()*param_.ActionSetSize()==0)
    {
      LERROR("invalid parameter vector.");
      LDBGVAR(get_feature_dim()); LDBGVAR(get_action_set_size());
      lexit(df);
    }
  }
}
//-------------------------------------------------------------------------------------------

override void MAVFLinearDiscActionB::slot_reset_exec (void)
{
}
//-------------------------------------------------------------------------------------------

/*!\brief update the parameter from the training sample data. */
override void MAVFLinearDiscActionB::slot_train_with_data_exec (const std::list<TActionValueFuncTrainingSample<TState,TAction> > &data)
{
  typedef TActionValueFuncTrainingSample<TState,TAction> TS;
  int num_act(param_.ActionSetSize()), num_bf(param_.BFSize());

  std::vector<int> num_of_data(num_act,0);
  for(std::list<TS>::const_iterator itr(data.begin()),last(data.end()); itr!=last; ++itr)
    ++num_of_data[itr->Action];

  // for each action, update the parameter param_.Theta[a]
  for(int a(0); a<num_act; ++a)
  {
    int num_dat(num_of_data[a]);
    if(num_dat==0)  continue;

    // setup the training data {F,V} for each action
    TRealMatrix F(num_dat, num_bf, 0.0);
    TRealVector V(num_dat, 0.0);
    int n(0);
    for(std::list<TS>::const_iterator itr(data.begin()),last(data.end()); itr!=last; ++itr)
    {
      if(itr->Action!=a)  continue;
      for(int i(0); i<num_bf; ++i)  F(n,i)= itr->State(i);
      V(n)= itr->Q;
      ++n;
    }

    // update param_.Theta[a]
    const TReal &lambda(conf_.RegularizationCoefficient);
    param_.Theta[a]= (F.transpose()*F+lambda*GetEye(num_bf)).inverse()*F.transpose() * V;
  }
}
//-------------------------------------------------------------------------------------------

// NOTE: the same as MAVFLinearDiscAction (FIXME: define a base class)
override void MAVFLinearDiscActionB::out_evaluate_get (const TState &x, const TAction &a, TStateActionAttribute attrib) const
{
  if (attrib.IsNull())  return;

  /// select action

  if (attrib.StateValue)
  {
    const TBoolVector *action_availability_s(NULL);
    if (in_action_availability_s.ConnectionSize()>0)
    {
      action_availability_s= &(in_action_availability_s.GetFirst(x));
      LASSERT1op1((int)action_availability_s->size(),==,get_action_set_size());
    }

    nextQs.resize (get_action_set_size(),0.0l);
    calc_action_value (param_.Theta, nextQs, x);

    #define NEXTQS          GenBegin(nextQs), GenEnd(nextQs)
    #define NEXTACTAVL      GenBegin(*action_availability_s), GenEnd(*action_availability_s)
    if (action_availability_s==NULL)  *attrib.StateValue= disc_action::StateValue (NEXTQS);
    else                              *attrib.StateValue= disc_action::StateValue (NEXTQS, NEXTACTAVL);
    if (attrib.ActionValue)  *attrib.ActionValue= nextQs(a);
    #undef NEXTQS
    #undef NEXTACTAVL
  }
  else
  {
    if (attrib.ActionValue)
      *attrib.ActionValue= InnerProd(GenBegin(param_.Theta[a]),GenEnd(param_.Theta[a]),GenBegin(x));
  }

  ////////////////////////////////////////////////////////////////////////////
  if (attrib.Gradient==NULL)  return;
  TAVFLinearDiscActionBParameter *grad= dynamic_cast<TAVFLinearDiscActionBParameter *>(attrib.Gradient);
  if (grad==NULL)  {LERROR("type conversion error in MAVFLinearDiscActionB"); lexit(df);}

  /// gradient
  grad->Init (param_.BFSize(), param_.ActionSetSize());
  grad->Theta[a]= x;
}
//-------------------------------------------------------------------------------------------

// NOTE: the same as MAVFLinearDiscAction (FIXME: define a base class)
override void MAVFLinearDiscActionB::out_greedy_get (const TState &x, TAction *greedy, TStateActionAttribute attrib) const
{
  if (greedy==NULL && attrib.IsNull())  return;

  const TBoolVector *action_availability_s(NULL);
  if (in_action_availability_s.ConnectionSize()>0)
  {
    action_availability_s= &(in_action_availability_s.GetFirst(x));
    LASSERT1op1((int)action_availability_s->size(),==,get_action_set_size());
  }

  /// select action
  TAction  next_action;

  nextQs.resize (get_action_set_size(),0.0l);
  next_policy.resize (get_action_set_size(),0.0l);

  // action value calculation
  calc_action_value (param_.Theta, nextQs, x);

  // policy calculation
  #define NEXTQS          GenBegin(nextQs), GenEnd(nextQs)
  #define NEXTPOLICY      GenBegin(next_policy), GenEnd(next_policy)
  #define NEXTACTAVL      GenBegin(*action_availability_s), GenEnd(*action_availability_s)
  if (action_availability_s==NULL)  next_action = disc_action::Greedy (NEXTQS, NEXTPOLICY);
  else                              next_action = disc_action::Greedy (NEXTQS, NEXTPOLICY, NEXTACTAVL);

  if (next_action<0)
    {LERROR(ModuleUniqueCode()<<": the current version does not allow to take an invalid action "<<next_action<<". "
      "instead, define a `do nothing\' action."); lexit(df);}

  if (greedy)  *greedy= next_action;
  if (attrib.ActionValue)  *attrib.ActionValue= nextQs(next_action);
  if (attrib.StateValue)
  {
    if (action_availability_s==NULL)  *attrib.StateValue= disc_action::StateValue (NEXTQS);
    else                              *attrib.StateValue= disc_action::StateValue (NEXTQS, NEXTACTAVL);
  }
  #undef NEXTQS
  #undef NEXTPOLICY
  #undef NEXTACTAVL

  ////////////////////////////////////////////////////////////////////////////
  if (attrib.Gradient==NULL)  return;
  TAVFLinearDiscActionBParameter *grad= dynamic_cast<TAVFLinearDiscActionBParameter *>(attrib.Gradient);
  if (grad==NULL)  {LERROR("type conversion error in MAVFLinearDiscActionB"); lexit(df);}

  /// gradient
  grad->Init (param_.BFSize(), param_.ActionSetSize());
  grad->Theta[next_action]= x;
}
//-------------------------------------------------------------------------------------------

// NOTE: the same as MAVFLinearDiscAction (FIXME: define a base class)
override void MAVFLinearDiscActionB::out_select_action_get (TAction *a, TStateActionAttribute attrib) const
{
  if (a==NULL && attrib.IsNull())  return;

  next_feature= get_feature();

  const TBoolVector *action_availability(NULL);
  if (in_action_availability.ConnectionSize()>0)
  {
    action_availability= &(in_action_availability.GetFirst());
    LASSERT1op1((int)action_availability->size(),==,get_action_set_size());
  }

  /// select action
  TAction  next_action(-1);
  bool  is_greedy(false);

  nextQs.resize (get_action_set_size(),0.0l);
  next_policy.resize (get_action_set_size(),0.0l);

  // action value calculation
  calc_action_value (param_.Theta, nextQs, next_feature);

  // policy calculation
  #define NEXTQS          GenBegin(nextQs), GenEnd(nextQs)
  #define NEXTPOLICY      GenBegin(next_policy), GenEnd(next_policy)
  #define NEXTACTAVL      GenBegin(*action_availability), GenEnd(*action_availability)
  if (action_availability==NULL)
  {
    if (conf_.ActionSelection==disc_action::asGreedy)
    {
      next_action = disc_action::Greedy (NEXTQS, NEXTPOLICY);
      is_greedy= true;
    }
    else if (conf_.ActionSelection==disc_action::asEpsGreedy)
      disc_action::EpsGreedy (get_eps(), NEXTQS, NEXTPOLICY);
    else if (conf_.ActionSelection==disc_action::asBoltzman)
      disc_action::Boltzmann  (get_tau(), NEXTQS, NEXTPOLICY);
    else if (conf_.ActionSelection==disc_action::asHeuristicWeightedBoltzman)
    {
      FIXME("ActionSelection==asHeuristicWeightedBoltzman is not implemented in this version yet..");
      // calcHeuristicBoltzmanWeight (x, hbweight, &SCACHE);
      // WeightedBoltzmann ((next_tau=get_tau()), NEXTQS,
              // hbweight.begin(), hbweight.end(),
              // NEXTPOLICY);
    }
  }
  else  // i.e. action_availability is NOT NULL
  {
    if (conf_.ActionSelection==disc_action::asGreedy)
    {
      next_action = disc_action::Greedy (NEXTQS, NEXTPOLICY, NEXTACTAVL);
      is_greedy= true;
    }
    else if (conf_.ActionSelection==disc_action::asEpsGreedy)
      disc_action::EpsGreedy (get_eps(), NEXTQS, NEXTPOLICY, NEXTACTAVL);
    else if (conf_.ActionSelection==disc_action::asBoltzman)
      disc_action::Boltzmann  (get_tau(), NEXTQS, NEXTPOLICY, NEXTACTAVL);
    else if (conf_.ActionSelection==disc_action::asHeuristicWeightedBoltzman)
    {
      FIXME("ActionSelection==asHeuristicWeightedBoltzman is not implemented in this version yet..");
      // calcHeuristicBoltzmanWeight (x, hbweight, &SCACHE);
      // WeightedBoltzmann ((next_tau=get_tau()), NEXTQS,
              // hbweight.begin(), hbweight.end(),
              // NEXTPOLICY, NEXTACTAVL);
    }
  }

  if (!is_greedy)
    next_action= disc_action::SelectActionFromPolicy(GenBegin(next_policy),GenEnd(next_policy));

  if (next_action<0)
    {LERROR(ModuleUniqueCode()<<": the current version does not allow to take an invalid action "<<next_action<<". "
      "instead, define a `do nothing\' action."); lexit(df);}

  if (a)  *a= next_action;
  if (attrib.ActionValue)  *attrib.ActionValue= nextQs(next_action);
  if (attrib.StateValue)
  {
    if (action_availability==NULL)  *attrib.StateValue= disc_action::StateValue (NEXTQS);
    else                            *attrib.StateValue= disc_action::StateValue (NEXTQS, NEXTACTAVL);
    // {
      // TValue qmax(-0.12345);
      // bool is_first(true);
      // TypeExtS<TBoolVector>::const_iterator  actavlitr(GenBegin(*action_availability));
      // for (TypeExtS<TRealVector>::const_iterator qitr(GenBegin(nextQs)); qitr!=GenEnd(nextQs); ++qitr,++actavlitr)
        // if (*actavlitr) {if (is_first || *qitr>qmax) {qmax=*qitr; is_first=false;}}
      // *attrib.StateValue= qmax;
    // }
  }
  #undef NEXTQS
  #undef NEXTPOLICY
  #undef NEXTACTAVL

  ////////////////////////////////////////////////////////////////////////////
  if (attrib.Gradient==NULL)  return;
  TAVFLinearDiscActionBParameter *grad= dynamic_cast<TAVFLinearDiscActionBParameter *>(attrib.Gradient);
  if (grad==NULL)  {LERROR("type conversion error in MAVFLinearDiscActionB"); lexit(df);}

  /// gradient
  grad->Init (param_.BFSize(), param_.ActionSetSize());
  grad->Theta[next_action]= next_feature;
}
//-------------------------------------------------------------------------------------------

// NOTE: the same as MAVFLinearDiscAction (FIXME: define a base class)
inline TReal MAVFLinearDiscActionB::get_eps () const
{
  switch (conf_.PolicyImprovement)
  {
    case piConst         :
          return conf_.Eps;
    case piExpReduction  :
          return conf_.Eps * real_exp (-conf_.EpsDecreasingFactor * static_cast<TReal>(get_episode_number()));
    default  :  LERROR("invalid PolicyImprovement "<<static_cast<int>(conf_.PolicyImprovement)); lexit(df);
  }
  return -1.0l;
}
//-------------------------------------------------------------------------------------------
// NOTE: the same as MAVFLinearDiscAction (FIXME: define a base class)
inline TReal MAVFLinearDiscActionB::get_tau () const
{
  TReal tau (0.0l);
  switch (conf_.PolicyImprovement)
  {
    case piConst              :
          tau= conf_.Tau;
          break;
    case piExpReduction       :
          tau= conf_.Tau * real_exp (-conf_.TauDecreasingFactor * static_cast<TReal>(get_episode_number()));
          break;
    default  :  LERROR("invalid PolicyImprovement "<<static_cast<int>(conf_.PolicyImprovement)); lexit(df);
  }
  return ApplyRange(tau,conf_.TauMin,conf_.Tau);
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
SKYAI_ADD_MODULE(MAVFLinearDiscActionB)
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
}  // end of avf_linear_discaction_b_detail
}  // end of loco_rabbits
//-------------------------------------------------------------------------------------------
