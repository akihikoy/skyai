//-------------------------------------------------------------------------------------------
/*! \file    avf_table.cpp
    \brief   libskyai - table-lookup function approximator module for action value function over discrete state-action space (source)
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

    \note  MAVFTable is very similar to MAVFLinearDiscAction
*/
//-------------------------------------------------------------------------------------------
#include <skyai/modules_std/avf_table.h>
#include <lora/stl_ext.h>
#include <lora/stl_math.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
namespace avf_table_detail
{
using namespace std;
// using namespace boost;


/*!\brief calculate action value at x with action value table Q, and store in resQs */
static void calc_action_value (const std::vector<TRealVector> &Q, TRealVector &resQs, const TDiscreteState &x)
{
  double *qs_itr(GenBegin(resQs));
  for (std::vector<TRealVector>::const_iterator qitr(Q.begin()); qitr!=Q.end(); ++qitr, ++qs_itr)
    *qs_itr= (*qitr)(x);
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class MAVFTable
//===========================================================================================

override void MAVFTable::slot_initialize_exec (void)
{
  if (param_.BFSize()*param_.ActionSetSize()==0)
  {
    // parameter initialization...

    param_.Init (get_state_set_size(), get_action_set_size());

    if (param_.BFSize()*param_.ActionSetSize()==0)
    {
      LERROR("invalid parameter vector.");
      LDBGVAR(get_state_set_size()); LDBGVAR(get_action_set_size());
      lexit(df);
    }
  }
}
//-------------------------------------------------------------------------------------------

override void MAVFTable::slot_reset_exec (void)
{
}
//-------------------------------------------------------------------------------------------

override void MAVFTable::slot_add_to_parameter_exec (const TParameter &diff)
{
  param_+= diff;
}
//-------------------------------------------------------------------------------------------


override const MAVFTable::TParameter&
    MAVFTable::out_parameter_ref_get (void) const
{
  return param_;
}
//-------------------------------------------------------------------------------------------

override void MAVFTable::out_parameter_val_get (TParameter &outerparam) const
{
  outerparam= param_;
}
//-------------------------------------------------------------------------------------------

override void MAVFTable::out_evaluate_get (const TState &x, const TAction &a, TStateActionAttribute attrib) const
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
      *attrib.ActionValue= (param_.Theta[a])(x);
  }

  ////////////////////////////////////////////////////////////////////////////
  if (attrib.Gradient==NULL)  return;
  TAVFTableParameter *grad= dynamic_cast<TAVFTableParameter *>(attrib.Gradient);
  if (grad==NULL)  {LERROR("type conversion error in MAVFTable"); lexit(df);}

  /// gradient
  grad->Init (param_.BFSize(), param_.ActionSetSize());
  grad->Theta[a](x)= 1.0l;
}
//-------------------------------------------------------------------------------------------

override void MAVFTable::out_greedy_get (const TState &x, TAction *greedy, TStateActionAttribute attrib) const
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
  TAVFTableParameter *grad= dynamic_cast<TAVFTableParameter *>(attrib.Gradient);
  if (grad==NULL)  {LERROR("type conversion error in MAVFTable"); lexit(df);}

  /// gradient
  grad->Init (param_.BFSize(), param_.ActionSetSize());
  grad->Theta[next_action](x)= 1.0l;
}
//-------------------------------------------------------------------------------------------

override void MAVFTable::out_select_action_get (TAction *a, TStateActionAttribute attrib) const
{
  if (a==NULL && attrib.IsNull())  return;

  next_state= get_state();

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
  calc_action_value (param_.Theta, nextQs, next_state);

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
  TAVFTableParameter *grad= dynamic_cast<TAVFTableParameter *>(attrib.Gradient);
  if (grad==NULL)  {LERROR("type conversion error in MAVFTable"); lexit(df);}

  /// gradient
  grad->Init (param_.BFSize(), param_.ActionSetSize());
  grad->Theta[next_action](next_state)= 1.0l;
}
//-------------------------------------------------------------------------------------------

override void MAVFTable::out_replacing_trace_get (TParameter &eligibility_trace) const
{
  TAVFTableParameter *p_eligibility_trace
      = dynamic_cast<TAVFTableParameter *>(&eligibility_trace);
  if (p_eligibility_trace==NULL)
    {LERROR("type conversion error in TAVFTableParameter"); lexit(df);}

  //!\todo FIXME: wasting computational resource!!
  for (std::vector<TRealVector>::iterator  vqitr(p_eligibility_trace->Theta.begin()),
          vqitr_last(p_eligibility_trace->Theta.end());
          vqitr!=vqitr_last; ++vqitr)
  {
    for (double *vqaitr(GenBegin(*vqitr)), *vqaitr_last(GenEnd(*vqitr)); vqaitr!=vqaitr_last; ++vqaitr)
      if (*vqaitr > conf_.TraceMax)  *vqaitr= conf_.TraceMax;
  }
}
//-------------------------------------------------------------------------------------------

override MAVFTable::TParameter*
    MAVFTable::out_create_parameter_get (void) const
{
  TAVFTableParameter *p = new TAVFTableParameter();
  if (param_.BFSize()*param_.ActionSetSize()==0)
    p->Init (get_state_set_size(), get_action_set_size());
  else
    p->Init (param_.BFSize(), param_.ActionSetSize());
  return p;
}
//-------------------------------------------------------------------------------------------

override void MAVFTable::out_zero_parameter_get (TParameter &outerparam) const
{
  TAVFTableParameter *prhs= dynamic_cast<TAVFTableParameter *>(&outerparam);
  if (prhs==NULL)  {LERROR("type conversion error in TAVFTableParameter"); lexit(df);}
  prhs->Zero();
}
//-------------------------------------------------------------------------------------------


inline TReal MAVFTable::get_eps () const
{
  switch (conf_.PolicyImprovement)
  {
    case avf_linear_discaction_detail::piConst         :
          return conf_.Eps;
    case avf_linear_discaction_detail::piExpReduction  :
          return conf_.Eps * real_exp (-conf_.EpsDecreasingFactor * static_cast<TReal>(get_episode_number()));
    default  :  LERROR("invalid PolicyImprovement "<<static_cast<int>(conf_.PolicyImprovement)); lexit(df);
  }
  return -1.0l;
}
//-------------------------------------------------------------------------------------------
inline TReal MAVFTable::get_tau () const
{
  TReal tau (0.0l);
  switch (conf_.PolicyImprovement)
  {
    case avf_linear_discaction_detail::piConst              :
          tau= conf_.Tau;
          break;
    case avf_linear_discaction_detail::piExpReduction       :
          tau= conf_.Tau * real_exp (-conf_.TauDecreasingFactor * static_cast<TReal>(get_episode_number()));
          break;
    default  :  LERROR("invalid PolicyImprovement "<<static_cast<int>(conf_.PolicyImprovement)); lexit(df);
  }
  return ApplyRange(tau,conf_.TauMin,conf_.Tau);
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
SKYAI_ADD_MODULE(MAVFTable)
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
}  // end of avf_table_detail
}  // end of loco_rabbits
//-------------------------------------------------------------------------------------------
