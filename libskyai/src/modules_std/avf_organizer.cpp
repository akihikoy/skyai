//-------------------------------------------------------------------------------------------
/*! \file    avf_organizer.cpp
    \brief   libskyai - function approximator organizer module for action value function (source)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Dec.21, 2009-

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
#include <skyai/modules_std/avf_organizer.h>
#include <boost/bind.hpp>
#include <lora/stl_ext.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
namespace avf_organizer_detail
{
using namespace std;
// using namespace boost;




//===========================================================================================
// class TAVFOrganizerParameter
//===========================================================================================


//!\brief assign zero (but size is not changed)
override void TAVFOrganizerParameter::Zero (void)
{
  for (TLowerParamSet::iterator itr(param_set_.begin()); itr!=param_set_.end(); ++itr)
  {
    itr->Param().Zero();
    itr->IsZero= true;
  }
}
//-------------------------------------------------------------------------------------------

//!\brief return the norm of the parameter
override TReal TAVFOrganizerParameter::Norm (void) const
{
  TReal n(0.0l);

  TLowerParamSet::const_iterator   lpitr (param_set_.begin()), lpitr_last (param_set_.end());

  for (; lpitr!=lpitr_last; ++lpitr)
  {
    if (lpitr->IsZero)  continue;
    else                n+= Square(lpitr->Param().Norm());
  }

  return real_sqrt(n);
}
//-------------------------------------------------------------------------------------------

/*!\brief return (*this = rhs) */
override const TActionValueFuncParamInterface&
    TAVFOrganizerParameter::operator= (const TActionValueFuncParamInterface &rhs)
{
  const TAVFOrganizerParameter *prhs= dynamic_cast<const TAVFOrganizerParameter *>(&rhs);
  if (prhs==NULL)  {LERROR("type conversion error in TAVFOrganizerParameter"); lexit(df);}

  if (param_set_.size() != prhs->param_set_.size())
    {LERROR("size disagreement!");  LDBGVAR(param_set_.size()); LDBGVAR(prhs->param_set_.size()); lexit(df);}

  TLowerParamSet::iterator          lpitr (param_set_.begin()), lpitr_last (param_set_.end());
  TLowerParamSet::const_iterator    rpitr (prhs->param_set_.begin());

  for (; lpitr!=lpitr_last; ++lpitr, ++rpitr)
  {
    if (rpitr->IsZero)
    {
      if (lpitr->IsZero)  continue;
      else
      {
        lpitr->Param().Zero();
        lpitr->IsZero= true;
      }
    }
    else
    {
      lpitr->Param= rpitr->Param;
      lpitr->IsZero= false;
    }
  }
  return *this;
}
//-------------------------------------------------------------------------------------------

/*!\brief return (*this += rhs) */
override const TActionValueFuncParamInterface&
    TAVFOrganizerParameter::operator+= (const TActionValueFuncParamInterface &rhs)
{
  const TAVFOrganizerParameter *prhs= dynamic_cast<const TAVFOrganizerParameter *>(&rhs);
  if (prhs==NULL)  {LERROR("type conversion error in TAVFOrganizerParameter"); lexit(df);}

  if (param_set_.size() != prhs->param_set_.size())
    {LERROR("size disagreement!");  LDBGVAR(param_set_.size()); LDBGVAR(prhs->param_set_.size()); lexit(df);}

  TLowerParamSet::iterator          lpitr (param_set_.begin()), lpitr_last (param_set_.end());
  TLowerParamSet::const_iterator    rpitr (prhs->param_set_.begin());

  for (; lpitr!=lpitr_last; ++lpitr, ++rpitr)
  {
    if (rpitr->IsZero)  continue;
    else
    {
      lpitr->Param()+= rpitr->Param();
      lpitr->IsZero= false;
    }
  }
  return *this;
}
//-------------------------------------------------------------------------------------------

//!\brief return (*this += weight*rhs)
override const TActionValueFuncParamInterface&
    TAVFOrganizerParameter::AddProd (const TReal &weight, const TActionValueFuncParamInterface &rhs)
{
  const TAVFOrganizerParameter *prhs= dynamic_cast<const TAVFOrganizerParameter *>(&rhs);
  if (prhs==NULL)  {LERROR("type conversion error in TAVFOrganizerParameter"); lexit(df);}

  if (param_set_.size() != prhs->param_set_.size())
    {LERROR("size disagreement!");  LDBGVAR(param_set_.size()); LDBGVAR(prhs->param_set_.size()); lexit(df);}

  TLowerParamSet::iterator          lpitr (param_set_.begin()), lpitr_last (param_set_.end());
  TLowerParamSet::const_iterator    rpitr (prhs->param_set_.begin());

  for (; lpitr!=lpitr_last; ++lpitr, ++rpitr)
  {
    if (rpitr->IsZero)  continue;
    else
    {
      lpitr->Param().AddProd(weight, rpitr->Param());
      lpitr->IsZero= false;
    }
  }
  return *this;
}
//-------------------------------------------------------------------------------------------

/*!\brief return (*this *= rhs) */
override const TActionValueFuncParamInterface&
    TAVFOrganizerParameter::operator*= (const TReal &rhs)
{
  TLowerParamSet::iterator          lpitr (param_set_.begin()), lpitr_last (param_set_.end());

  for (; lpitr!=lpitr_last; ++lpitr)
  {
    if (lpitr->IsZero)  continue;
    else                lpitr->Param()*= rhs;
  }
  return *this;
}
//-------------------------------------------------------------------------------------------



//===========================================================================================
// class MAVFOrganizer
//===========================================================================================

override void MAVFOrganizer::slot_initialize_exec (void)
{
  make_lowerc_map();
  // init_parameter (param_);

  // construct cache_select_info_
  cache_select_info_.resize (conf_.OrderOfLower.size());
  for (size_t m(0); m<cache_select_info_.size(); ++m)
  {
    cache_select_info_[m].Grad.SetAllocator (
        boost::bind (&GET_PORT_TYPE(in_avf_create_parameter)::GetCurrent,
                      &in_avf_create_parameter,
                      lowerc_map_[m].in_avf_create_parameter_citr()) );
    cache_select_info_[m].Grad.Allocate();
  }
}
//-------------------------------------------------------------------------------------------

override void MAVFOrganizer::slot_reset_exec (void)
{
  // next_action_.Index= -1;
}
//-------------------------------------------------------------------------------------------

override void MAVFOrganizer::slot_add_to_parameter_exec (const TParameter &diff)
{
  const TAVFOrganizerParameter *pdiff= dynamic_cast<const TAVFOrganizerParameter *>(&diff);
  if (pdiff==NULL)  {LERROR("type conversion error in MAVFOrganizer"); lexit(df);}

  if (static_cast<int>(lowerc_map_.size()) != pdiff->Size())
    {LERROR("size disagreement!");  LDBGVAR(lowerc_map_.size()); LDBGVAR(pdiff->Size()); lexit(df);}

  for (int i(0), ilast(pdiff->Size()); i<ilast; ++i)
  {
    if (!pdiff->IsZero(i))
      signal_avf_add_to_parameter.ExecCurrent (lowerc_map_[i].signal_avf_add_to_parameter_citr(), pdiff->Param(i)());
  }
}
//-------------------------------------------------------------------------------------------


override const MAVFOrganizer::TParameter&
    MAVFOrganizer::out_parameter_ref_get (void) const
{
  LERROR("it is not allowed to use the out_parameter_ref port of the MAVFOrganizer module");
  return dummy_return<MAVFOrganizer::TParameter&>::value();
}
//-------------------------------------------------------------------------------------------

override void MAVFOrganizer::out_parameter_val_get (TParameter &outerparam) const
{
  FIXME("the following code is not debugged (maybe no problem)");
  // const TAVFOrganizerParameter *pouterparam= dynamic_cast<const TAVFOrganizerParameter *>(&outerparam);
  // if (pouterparam==NULL)  {LERROR("type conversion error in MAVFOrganizer"); lexit(df);}

  // init_parameter (*pouterparam);

  // for (int i(0), ilast(pdiff->Size()); i<ilast; ++i)
  // {
    // if (!pdiff->IsZero(i))
      // in_avf_parameter_val.ExecCurrent (lowerc_map_[i].in_avf_parameter_val_citr(), pouterparam->SetParam(i));
  // }
}
//-------------------------------------------------------------------------------------------


override void MAVFOrganizer::out_evaluate_get (const TState &x, const TAction &a, TStateActionAttribute attrib) const
{
  if (attrib.IsNull())  return;

  TInt lower_idx= a.DiscSet[0];
  TInt m= lower_idx;

  TStateActionAttribute lower_attrib(&(cache_select_info_[m].Q), &(cache_select_info_[m].V), (attrib.Gradient?&(cache_select_info_[m].Grad()):NULL) );
  TInt state_idx= lower_idx_to_state_idx_[m];
  if (lowerc_map_[m].StateType==satDiscrete)
  {
    if (lowerc_map_[m].ActionType==satDiscrete)
      in_avf_evaluate_dd.GetCurrent (lowerc_map_[m].in_avf_evaluate_dd_citr(),
                  x.DiscSet[state_idx], a.DiscSet[1], lower_attrib);
    else if (lowerc_map_[m].ActionType==satContinuous)
      in_avf_evaluate_dc.GetCurrent (lowerc_map_[m].in_avf_evaluate_dc_citr(),
                  x.DiscSet[state_idx], a.ContSet[0], lower_attrib);
    else {LERROR("fatal!"); LDBGVAR(lowerc_map_[m].LowerModuleName); LDBGVAR(lowerc_map_[m].ActionType); lexit(df);}
  }
  else if (lowerc_map_[m].StateType==satContinuous)
  {
    if (lowerc_map_[m].ActionType==satDiscrete)
      in_avf_evaluate_cd.GetCurrent (lowerc_map_[m].in_avf_evaluate_cd_citr(),
                  x.ContSet[state_idx], a.DiscSet[1], lower_attrib);
    else if (lowerc_map_[m].ActionType==satContinuous)
      in_avf_evaluate_cc.GetCurrent (lowerc_map_[m].in_avf_evaluate_cc_citr(),
                  x.ContSet[state_idx], a.ContSet[0], lower_attrib);
    else {LERROR("fatal!"); LDBGVAR(lowerc_map_[m].LowerModuleName); LDBGVAR(lowerc_map_[m].ActionType); lexit(df);}
  }
  else {LERROR("fatal!"); LDBGVAR(lowerc_map_[m].LowerModuleName); LDBGVAR(lowerc_map_[m].StateType); lexit(df);}

  if (attrib.ActionValue)  *attrib.ActionValue= cache_select_info_[m].Q;

  ////////////////////////////////////////////////////////////////////////////
  if (attrib.Gradient==NULL)  return;
  TAVFOrganizerParameter *grad= dynamic_cast<TAVFOrganizerParameter *>(attrib.Gradient);
  if (grad==NULL)  {LERROR("type conversion error in MAVFOrganizer"); lexit(df);}

  /// gradient
  init_parameter (*grad);
  grad->SetParam(m)= cache_select_info_[m].Grad;
  ////////////////////////////////////////////////////////////////////////////

  if (attrib.StateValue)
  {
    LWARNING("fix this code (taking a lot of computational cost)");
    TStateActionAttribute tmp_attrib(NULL, attrib.StateValue, NULL);
    out_greedy_get(x, NULL, tmp_attrib);
  }
}
//-------------------------------------------------------------------------------------------

override void MAVFOrganizer::out_greedy_get (const TState &x, TAction *greedy, TStateActionAttribute attrib) const
{
  if (greedy==NULL && attrib.IsNull())  return;

  int  Nm (lowerc_map_.size());  // number of modules
  TInt  situation (x.DiscSet[0]);

  /// select action
  //!\todo FIXME: wasting computational resource!! (comuting all gradient!!)
  TInt  next_module;

  nextQs.resize (Nm,0.0l);
  nextVs.resize (Nm,0.0l);

  cache_available_mods_.resize (Nm,-1);

  TInt  Navailable (0);  // number of available modules
  for (int m(0); m<Nm; ++m)
  {
    if (!lowerc_map_[m].AvailableSituations[situation])  continue;

    TStateActionAttribute lower_attrib(&(cache_select_info_[m].Q), &(cache_select_info_[m].V), (attrib.Gradient?&(cache_select_info_[m].Grad()):NULL) );
    TInt state_idx= lower_idx_to_state_idx_[m];
    if (lowerc_map_[m].StateType==satDiscrete)
    {
      if (lowerc_map_[m].ActionType==satDiscrete)
        in_avf_greedy_dd.GetCurrent (lowerc_map_[m].in_avf_greedy_dd_citr(),
                    x.DiscSet[state_idx], &(cache_select_info_[m].Disc), lower_attrib);
      else if (lowerc_map_[m].ActionType==satContinuous)
        in_avf_greedy_dc.GetCurrent (lowerc_map_[m].in_avf_greedy_dc_citr(),
                    x.DiscSet[state_idx], &(cache_select_info_[m].Cont), lower_attrib);
      else {LERROR("fatal!"); LDBGVAR(lowerc_map_[m].LowerModuleName); LDBGVAR(lowerc_map_[m].ActionType); lexit(df);}
    }
    else if (lowerc_map_[m].StateType==satContinuous)
    {
      if (lowerc_map_[m].ActionType==satDiscrete)
        in_avf_greedy_cd.GetCurrent (lowerc_map_[m].in_avf_greedy_cd_citr(),
                    x.ContSet[state_idx], &(cache_select_info_[m].Disc), lower_attrib);
      else if (lowerc_map_[m].ActionType==satContinuous)
        in_avf_greedy_cc.GetCurrent (lowerc_map_[m].in_avf_greedy_cc_citr(),
                    x.ContSet[state_idx], &(cache_select_info_[m].Cont), lower_attrib);
      else {LERROR("fatal!"); LDBGVAR(lowerc_map_[m].LowerModuleName); LDBGVAR(lowerc_map_[m].ActionType); lexit(df);}
    }
    else {LERROR("fatal!"); LDBGVAR(lowerc_map_[m].LowerModuleName); LDBGVAR(lowerc_map_[m].StateType); lexit(df);}
    cache_available_mods_[Navailable]= m;
    nextQs(Navailable)= cache_select_info_[m].Q;
    nextVs(Navailable)= cache_select_info_[m].V;
    ++Navailable;
  }

  /*greedy selection*/{
    TInt m= max_element_index (GenBegin(nextQs), GenBegin(nextQs)+Navailable);
    next_module = cache_available_mods_[m];
  }

  if (greedy)
  {
    if (lowerc_map_[next_module].ActionType==satDiscrete)
    {
      greedy->DiscSet.resize(2);
      greedy->ContSet.resize(0);
      greedy->DiscSet[0]= next_module;
      greedy->DiscSet[1]= cache_select_info_[next_module].Disc;
    }
    else if (lowerc_map_[next_module].ActionType==satContinuous)
    {
      greedy->DiscSet.resize(1);
      greedy->ContSet.resize(1);
      greedy->DiscSet[0]= next_module;
      greedy->ContSet[0]= cache_select_info_[next_module].Cont;
    }
  }
  if (attrib.ActionValue)  *attrib.ActionValue= cache_select_info_[next_module].Q;
  // if (attrib.StateValue)  *attrib.StateValue= *std::max_element (GenBegin(nextVs), GenBegin(nextVs)+Navailable);
  if (attrib.StateValue)  *attrib.StateValue= cache_select_info_[next_module].Q;

  ////////////////////////////////////////////////////////////////////////////
  if (attrib.Gradient==NULL)  return;
  TAVFOrganizerParameter *grad= dynamic_cast<TAVFOrganizerParameter *>(attrib.Gradient);
  if (grad==NULL)  {LERROR("type conversion error in MAVFOrganizer"); lexit(df);}

  /// gradient
  init_parameter (*grad);
  grad->SetParam(next_module)= cache_select_info_[next_module].Grad;
}
//-------------------------------------------------------------------------------------------

override void MAVFOrganizer::out_select_action_get (TAction *a, TStateActionAttribute attrib) const
{
  if (a==NULL && attrib.IsNull())  return;

  int  Nm (lowerc_map_.size());  // number of modules
  // const std::vector<bool>&  available_modules (get_available_modules());
  TInt  situation (get_situation());

  /// select action
  //!\todo FIXME: wasting computational resource!! (comuting all gradient!!)
  TInt  next_module;
  bool  greedy(false);

  nextQs.resize (Nm,0.0l);
  nextVs.resize (Nm,0.0l);
  next_policy.resize (Nm,0.0l);

  cache_available_mods_.resize (Nm,-1);
  cache_available_hbweights_.resize (Nm, -1.0l);

  TInt  Navailable (0);  // number of available modules
  for (int m(0); m<Nm; ++m)
  {
    // if (!available_modules[m])  continue;
    if (!lowerc_map_[m].AvailableSituations[situation])  continue;
    if (lowerc_map_[m].ActionType==satDiscrete)
    {
      if (attrib.Gradient)
        in_avf_select_action_d.GetCurrent (lowerc_map_[m].in_avf_select_action_d_citr(),
                    &(cache_select_info_[m].Disc), TStateActionAttribute(&(cache_select_info_[m].Q), &(cache_select_info_[m].V), &(cache_select_info_[m].Grad())));
      else
        in_avf_select_action_d.GetCurrent (lowerc_map_[m].in_avf_select_action_d_citr(),
                    &(cache_select_info_[m].Disc), TStateActionAttribute(&(cache_select_info_[m].Q), &(cache_select_info_[m].V), NULL));
    }
    else if (lowerc_map_[m].ActionType==satContinuous)
    {
      if (attrib.Gradient)
        in_avf_select_action_c.GetCurrent (lowerc_map_[m].in_avf_select_action_c_citr(),
                    &(cache_select_info_[m].Cont), TStateActionAttribute(&(cache_select_info_[m].Q), &(cache_select_info_[m].V), &(cache_select_info_[m].Grad())));
      else
        in_avf_select_action_c.GetCurrent (lowerc_map_[m].in_avf_select_action_c_citr(),
                    &(cache_select_info_[m].Cont), TStateActionAttribute(&(cache_select_info_[m].Q), &(cache_select_info_[m].V), NULL));
    }
    else {LERROR("fatal!"); LDBGVAR(lowerc_map_[m].LowerModuleName); LDBGVAR(lowerc_map_[m].ActionType); lexit(df);}
    cache_available_mods_[Navailable]= m;
    cache_available_hbweights_[Navailable]= lowerc_map_[m].HBWeight;
    nextQs(Navailable)= cache_select_info_[m].Q;
    nextVs(Navailable)= cache_select_info_[m].V;
    ++Navailable;
  }

  if (conf_.ActionSelection==disc_action::asGreedy)
  {
    TInt m= max_element_index (GenBegin(nextQs), GenBegin(nextQs)+Navailable);
    next_module = cache_available_mods_[m];
    std::fill (GenBegin(next_policy),GenEnd(next_policy),0.0l);
    next_policy(m)= 1.0l;
    greedy= true;
  }
  else if (conf_.ActionSelection==disc_action::asEpsGreedy)
    disc_action::EpsGreedy (get_eps(), GenBegin(nextQs), GenBegin(nextQs)+Navailable, GenBegin(next_policy), GenBegin(next_policy)+Navailable);
  else if (conf_.ActionSelection==disc_action::asBoltzman)
    disc_action::Boltzmann  (get_tau(), GenBegin(nextQs), GenBegin(nextQs)+Navailable, GenBegin(next_policy), GenBegin(next_policy)+Navailable);
  else if (conf_.ActionSelection==disc_action::asHeuristicWeightedBoltzman)
  {
    disc_action::WeightedBoltzmann (get_tau(), GenBegin(nextQs), GenBegin(nextQs)+Navailable,
            GenBegin(cache_available_hbweights_), GenBegin(cache_available_hbweights_)+Navailable,
            GenBegin(next_policy),GenBegin(next_policy)+Navailable);
  }

  if (!greedy)
    next_module= cache_available_mods_[
                  disc_action::SelectActionFromPolicy (GenBegin(next_policy),GenBegin(next_policy)+Navailable) ];

  // next_action_.Index++;
  // next_action_.LowerIndex= next_module;
  // next_action_.ActionType= lowerc_map_[next_module].ActionType;
  // if      (lowerc_map_[next_module].ActionType==satDiscrete)  next_action_.Disc= cache_select_info_[next_module].Disc;
  // else if (lowerc_map_[next_module].ActionType==satContinuous)  next_action_.Cont= cache_select_info_[next_module].Cont;
  // if (a)  *a= next_action_.Index;

  if (a)
  {
    if (lowerc_map_[next_module].ActionType==satDiscrete)
    {
      a->DiscSet.resize(2);
      a->ContSet.resize(0);
      a->DiscSet[0]= next_module;
      a->DiscSet[1]= cache_select_info_[next_module].Disc;
    }
    else if (lowerc_map_[next_module].ActionType==satContinuous)
    {
      a->DiscSet.resize(1);
      a->ContSet.resize(1);
      a->DiscSet[0]= next_module;
      a->ContSet[0]= cache_select_info_[next_module].Cont;
    }
  }
  if (attrib.ActionValue)  *attrib.ActionValue= cache_select_info_[next_module].Q;
  if (attrib.StateValue)  *attrib.StateValue= *std::max_element (GenBegin(nextVs), GenBegin(nextVs)+Navailable);

  ////////////////////////////////////////////////////////////////////////////
  if (attrib.Gradient==NULL)  return;
  TAVFOrganizerParameter *grad= dynamic_cast<TAVFOrganizerParameter *>(attrib.Gradient);
  if (grad==NULL)  {LERROR("type conversion error in MAVFOrganizer"); lexit(df);}

  /// gradient
  init_parameter (*grad);
  grad->SetParam(next_module)= cache_select_info_[next_module].Grad;
}
//-------------------------------------------------------------------------------------------

override void MAVFOrganizer::out_replacing_trace_get (TParameter &eligibility_trace) const
{
  TAVFOrganizerParameter *p_eligibility_trace
      = dynamic_cast<TAVFOrganizerParameter *>(&eligibility_trace);
  if (p_eligibility_trace==NULL)
    {LERROR("type conversion error in TAVFOrganizerParameter"); lexit(df);}

  if (static_cast<int>(lowerc_map_.size()) != p_eligibility_trace->Size())
    {LERROR("size disagreement!");  LDBGVAR(lowerc_map_.size()); LDBGVAR(p_eligibility_trace->Size()); lexit(df);}

  for (int i(0), ilast(p_eligibility_trace->Size()); i<ilast; ++i)
  {
    if (!p_eligibility_trace->IsZero(i))
      in_avf_replacing_trace.GetCurrent (lowerc_map_[i].in_avf_replacing_trace_citr(), p_eligibility_trace->SetParam(i)());
  }
}
//-------------------------------------------------------------------------------------------

override MAVFOrganizer::TParameter*
    MAVFOrganizer::out_create_parameter_get (void) const
{
  TAVFOrganizerParameter *p = new TAVFOrganizerParameter();
  init_parameter (*p);
  return p;
}
//-------------------------------------------------------------------------------------------

override void MAVFOrganizer::out_zero_parameter_get (TParameter &outerparam) const
{
  TAVFOrganizerParameter *prhs= dynamic_cast<TAVFOrganizerParameter *>(&outerparam);
  if (prhs==NULL)  {LERROR("type conversion error in TAVFOrganizerParameter"); lexit(df);}
  prhs->Zero();
}
//-------------------------------------------------------------------------------------------

/*!\brief return a current composite state \p x
    \note x.DiscSet[0] is in_situation.GetFirst() */
/*virtual*/const TCompositeState& MAVFOrganizer::out_composite_state_get (void) const
{
  tmp_state_.DiscSet.resize(disc_state_lowerc_size_+1);
  tmp_state_.ContSet.resize(cont_state_lowerc_size_);

  tmp_state_.DiscSet[0]= get_situation();
  TInt m(0);
  for (TLowerCMap::const_iterator itr(lowerc_map_.begin()), last(lowerc_map_.end()); itr!=last; ++itr,++m)
  {
    if (itr->StateType==satDiscrete)
      tmp_state_.DiscSet[lower_idx_to_state_idx_[m]]= in_avf_state_d.GetCurrent (itr->in_avf_state_d_citr());
    else if (itr->StateType==satContinuous)
      tmp_state_.ContSet[lower_idx_to_state_idx_[m]]= in_avf_state_c.GetCurrent (itr->in_avf_state_c_citr());
    else
      {LERROR(ModuleUniqueCode()<<" requires that an out-port of the current state of the lower module "
        <<itr->LowerModuleName<<" is connected to an in-port in_avf_state_*");  lexit(df);}
  }

  return tmp_state_;
}
//-------------------------------------------------------------------------------------------

/*!\brief execute a composite action \p a
    \note action \p a must be
      {a.DiscSet[0]:lower-module-index,
        a.DiscSet[1]:discrete-action (if the lower module is discrete action type),
        a.ContSet[0]:continuous-action (if the lower module is continuous action type)} */
void MAVFOrganizer::slot_execute_composite_action_exec (const TCompositeAction &a) const
{
  LASSERT1op1(a.DiscSet.size(),>=,1);
  TInt   lower_idx (a.DiscSet[0]);
  TStateActionType action_type (lowerc_map_[lower_idx].ActionType);
  if (action_type==satDiscrete)
  {
    LASSERT1op1(a.DiscSet.size(),>=,2);
    signal_execute_action_d.ExecCurrent (lowerc_map_[lower_idx].signal_execute_action_d_citr(), a.DiscSet[1]);
  }
  else if (action_type==satContinuous)
  {
    LASSERT1op1(a.ContSet.size(),>=,1);
    signal_execute_action_c.ExecCurrent (lowerc_map_[lower_idx].signal_execute_action_c_citr(), a.ContSet[0]);
  }
  else
    {LERROR("fatal!"); LDBGVAR((int)action_type); lexit(df);}
}
//-------------------------------------------------------------------------------------------

inline TReal MAVFOrganizer::get_eps () const
{
  switch (conf_.PolicyImprovement)
  {
    case piConst         :
          return conf_.Eps;
    case piExpReduction  :
          //return conf_.Eps * real_exp (-conf_.EpsDecreasingFactor * static_cast<TReal>(agent_result.total_goals()));
          return conf_.Eps * real_exp (-conf_.EpsDecreasingFactor * static_cast<TReal>(get_episode_number()));
    default  :  LERROR("invalid PolicyImprovement "<<static_cast<int>(conf_.PolicyImprovement)); lexit(df);
  }
  return -1.0l;
}
//-------------------------------------------------------------------------------------------
inline TReal MAVFOrganizer::get_tau () const
{
  TReal tau (0.0l);
  switch (conf_.PolicyImprovement)
  {
    case piConst              :
          tau= conf_.Tau;
          break;
    case piExpReduction       :
          //return conf_.Tau * real_exp (-conf_.TauDecreasingFactor * static_cast<TReal>(agent_result.total_goals()));
          tau= conf_.Tau * real_exp (-conf_.TauDecreasingFactor * static_cast<TReal>(get_episode_number()));
          break;
    default  :  LERROR("invalid PolicyImprovement "<<static_cast<int>(conf_.PolicyImprovement)); lexit(df);
  }
  return ApplyRange(tau,conf_.TauMin,conf_.Tau);
}
//-------------------------------------------------------------------------------------------


void MAVFOrganizer::make_lowerc_map (void)
{
  if (conf_.OrderOfLower.size() != conf_.OrderOfController.size() || conf_.OrderOfLower.size() != conf_.AvailableSituations.size())
    {LERROR("conf_.OrderOfLower.size() != conf_.OrderOfController.size() != conf_.AvailableSituations.size()");
      LDBGVAR(conf_.OrderOfLower.size()); LDBGVAR(conf_.OrderOfController.size());
      LDBGVAR(conf_.AvailableSituations.size()); lexit(df);}
  if (conf_.ActionSelection==disc_action::asHeuristicWeightedBoltzman && conf_.HBWeight.size() != conf_.OrderOfLower.size())
    {LERROR("conf_.ActionSelection==disc_action::asHeuristicWeightedBoltzman, but conf_.HBWeight.size() != conf_.OrderOfLower.size()");
      LDBGVAR(conf_.HBWeight.size()); LDBGVAR(conf_.OrderOfLower.size()); lexit(df);}

  lowerc_map_.clear();
  // size check...
  int lower_size (conf_.OrderOfLower.size());
  #define SIZE_CHECK(x_port) \
    if (x_port.ConnectionSize() != lower_size) \
      {LERROR("error! number of ports connected to "#x_port" should be the same as conf_.OrderOfLower.size()="<<lower_size); \
      lexit(df);}
  #define SIZE_CHECK_A(x_port) \
    if (x_port##_d.ConnectionSize() + x_port##_c.ConnectionSize() != lower_size) \
      {LERROR("error! number of ports connected to "#x_port"_{d,c} should be the same as conf_.OrderOfLower.size()="<<lower_size); \
      lexit(df);}
  SIZE_CHECK(signal_avf_add_to_parameter      )
  // SIZE_CHECK(in_avf_parameter_ref          )
  // SIZE_CHECK(in_avf_parameter_val          )
  // SIZE_CHECK_A(in_avf_evaluate             )
  // SIZE_CHECK_A(in_avf_greedy               )
  SIZE_CHECK_A(in_avf_select_action           )
  SIZE_CHECK(in_avf_replacing_trace           )
  SIZE_CHECK(in_avf_create_parameter          )
  SIZE_CHECK(in_avf_zero_parameter            )
  SIZE_CHECK_A(signal_execute_action          )
  #undef SIZE_CHECK

  TAVFOrganizerConfigurations::TOrderMap::const_iterator  order_itr;

  lowerc_map_.resize (lower_size);
  for (TLowerCMap::iterator itr(lowerc_map_.begin()); itr!=lowerc_map_.end(); ++itr)
  {
    itr->Outer= this;
    itr->ActionType= satUnknown;
    itr->StateType= satUnknown;
  }

  for (order_itr= conf_.OrderOfLower.begin(); order_itr!=conf_.OrderOfLower.end(); ++order_itr)
    lowerc_map_[order_itr->second].LowerModuleName= order_itr->first;
  /*set AvailableSituations*/{
    std::map<TString, std::vector<TInt> >  e_available_situations;
    std::vector<TInt>  situations;
    std::vector<TInt>::iterator tmpitr;
    int max_situation(0);
    for (TAVFOrganizerConfigurations::TStrStrMap::const_iterator itr(conf_.AvailableSituations.begin()); itr!=conf_.AvailableSituations.end(); ++itr)
    {
      situations= ConvertFromStr<std::vector<TInt> >(itr->second);
      e_available_situations[itr->first]= situations;
      tmpitr= std::max_element(situations.begin(), situations.end());
      if (tmpitr==situations.end())  continue;
      max_situation= std::max (max_situation, *tmpitr);
    }
    std::vector<bool>  available_situations (max_situation+1);
    for (std::map<TString, std::vector<TInt> >::const_iterator itr(e_available_situations.begin()); itr!=e_available_situations.end(); ++itr)
    {
      std::fill (available_situations.begin(), available_situations.end(), false);
      for (std::vector<TInt>::const_iterator sitr(itr->second.begin()); sitr!=itr->second.end(); ++sitr)
        available_situations[*sitr]= true;
      lowerc_map_[conf_.OrderOfLower[itr->first]].AvailableSituations= available_situations;
    }
  }

  /*set HBWeight*/{
    const TReal  invalid_weight (-1.0l);
    // clear HBWeight..
    for (TLowerCMap::iterator itr(lowerc_map_.begin()); itr!=lowerc_map_.end(); ++itr)
      itr->HBWeight= invalid_weight;
    if (conf_.ActionSelection==disc_action::asHeuristicWeightedBoltzman || conf_.HBWeight.size()==conf_.OrderOfLower.size())
    {
      for (TAVFOrganizerConfigurations::THBWeightMap::const_iterator itr(conf_.HBWeight.begin()); itr!=conf_.HBWeight.end(); ++itr)
        lowerc_map_[conf_.OrderOfLower[itr->first]].HBWeight= itr->second;
      // check..
      for (TLowerCMap::iterator itr(lowerc_map_.begin()); itr!=lowerc_map_.end(); ++itr)
        if (itr->HBWeight==invalid_weight)
          {LERROR("HBWeight for "<<itr->LowerModuleName<<" is not found"); lexit(df);}
    }
  }

  /*set iterators of port-connections to lower function approximators*/{
    #define REGISTER_A(x_port)  \
      for (GET_PORT_TYPE(x_port)::TConnectedPortIterator itr(x_port.ConnectedPortBegin()); itr!=x_port.ConnectedPortEnd(); ++itr) \
      {                                                                           \
        order_itr= conf_.OrderOfLower.find((*itr)->OuterBase().InstanceName());   \
        if (order_itr != conf_.OrderOfLower.end())                                \
          lowerc_map_[order_itr->second].x_port##_citr(itr);                      \
      }
    #define REGISTER_B(x_port,x_action_type)  \
      for (GET_PORT_TYPE(x_port)::TConnectedPortIterator itr(x_port.ConnectedPortBegin()); itr!=x_port.ConnectedPortEnd(); ++itr) \
      {                                                                           \
        order_itr= conf_.OrderOfLower.find((*itr)->OuterBase().InstanceName());   \
        if (order_itr != conf_.OrderOfLower.end())                                \
        {                                                                         \
          lowerc_map_[order_itr->second].x_port##_citr(itr);                      \
          lowerc_map_[order_itr->second].SetActionType(x_action_type);            \
        }                                                                         \
      }
    #define REGISTER_C(x_port,x_state_type)  \
      for (GET_PORT_TYPE(x_port)::TConnectedPortIterator itr(x_port.ConnectedPortBegin()); itr!=x_port.ConnectedPortEnd(); ++itr) \
      {                                                                           \
        order_itr= conf_.OrderOfLower.find((*itr)->OuterBase().InstanceName());   \
        if (order_itr != conf_.OrderOfLower.end())                                \
        {                                                                         \
          lowerc_map_[order_itr->second].x_port##_citr(itr);                      \
          lowerc_map_[order_itr->second].SetStateType(x_state_type);              \
        }                                                                         \
      }
    #define REGISTER_D(x_port,x_state_type,x_action_type)  \
      for (GET_PORT_TYPE(x_port)::TConnectedPortIterator itr(x_port.ConnectedPortBegin()); itr!=x_port.ConnectedPortEnd(); ++itr) \
      {                                                                           \
        order_itr= conf_.OrderOfLower.find((*itr)->OuterBase().InstanceName());   \
        if (order_itr != conf_.OrderOfLower.end())                                \
        {                                                                         \
          lowerc_map_[order_itr->second].x_port##_citr(itr);                      \
          lowerc_map_[order_itr->second].SetStateType(x_state_type);              \
          lowerc_map_[order_itr->second].SetActionType(x_action_type);            \
        }                                                                         \
      }
    #define REGISTER_CTRL(x_port,x_action_type)  \
      for (GET_PORT_TYPE(x_port)::TConnectedPortIterator itr(x_port.ConnectedPortBegin()); itr!=x_port.ConnectedPortEnd(); ++itr) \
      {                                                                   \
        order_itr= conf_.OrderOfController.find((*itr)->UniqueCode());    \
        if (order_itr != conf_.OrderOfController.end())                   \
        {                                                                 \
          lowerc_map_[order_itr->second].x_port##_citr(itr);              \
          lowerc_map_[order_itr->second].SetActionType(x_action_type);    \
        }                                                                 \
      }
    REGISTER_A(signal_avf_add_to_parameter )
    REGISTER_C(in_avf_state_d              , satDiscrete    )
    REGISTER_C(in_avf_state_c              , satContinuous  )
    // REGISTER_A(in_avf_parameter_ref        )
    // REGISTER_A(in_avf_parameter_val        )
    REGISTER_D(in_avf_evaluate_dd          , satDiscrete   , satDiscrete    )
    REGISTER_D(in_avf_evaluate_dc          , satDiscrete   , satContinuous  )
    REGISTER_D(in_avf_evaluate_cd          , satContinuous , satDiscrete    )
    REGISTER_D(in_avf_evaluate_cc          , satContinuous , satContinuous  )
    REGISTER_D(in_avf_greedy_dd            , satDiscrete   , satDiscrete    )
    REGISTER_D(in_avf_greedy_dc            , satDiscrete   , satContinuous  )
    REGISTER_D(in_avf_greedy_cd            , satContinuous , satDiscrete    )
    REGISTER_D(in_avf_greedy_cc            , satContinuous , satContinuous  )
    REGISTER_B(in_avf_select_action_d      , satDiscrete    )
    REGISTER_B(in_avf_select_action_c      , satContinuous  )
    REGISTER_A(in_avf_replacing_trace      )
    REGISTER_A(in_avf_create_parameter     )
    REGISTER_A(in_avf_zero_parameter       )
    REGISTER_CTRL(signal_execute_action_d  , satDiscrete    )
    REGISTER_CTRL(signal_execute_action_c  , satContinuous  )
    #undef REGISTER_CTRL
    #undef REGISTER_D
    #undef REGISTER_C
    #undef REGISTER_B
    #undef REGISTER_A
  }

  lower_idx_to_state_idx_.resize (lowerc_map_.size());
  disc_state_lowerc_size_= 0;
  cont_state_lowerc_size_= 0;
  TInt m(0);
  for (TLowerCMap::const_iterator itr(lowerc_map_.begin()), last(lowerc_map_.end()); itr!=last; ++itr, ++m)
  {
    if      (itr->StateType==satDiscrete)   {lower_idx_to_state_idx_[m]=disc_state_lowerc_size_; ++disc_state_lowerc_size_;}
    else if (itr->StateType==satContinuous) {lower_idx_to_state_idx_[m]=cont_state_lowerc_size_; ++cont_state_lowerc_size_;}
  }

}
//-------------------------------------------------------------------------------------------

void MAVFOrganizer::init_parameter (TAVFOrganizerParameter &outerparam, bool force_create) const
{
  if (lowerc_map_.size()!=conf_.OrderOfLower.size())
    {LERROR("execute slot_initialize before this part");}
  if (force_create || outerparam.Size()!=static_cast<int>(conf_.OrderOfLower.size()))
  {
    outerparam.Clear();
    outerparam.Resize (conf_.OrderOfLower.size());
    for (int m(0); m<outerparam.Size(); ++m)
    {
      outerparam.SetParam(m).SetAllocator (
          boost::bind (&GET_PORT_TYPE(in_avf_create_parameter)::GetCurrent,
                        &in_avf_create_parameter,
                        lowerc_map_[m].in_avf_create_parameter_citr()) );
      outerparam.SetParam(m).Allocate();
    }
  }
  outerparam.Zero();
}
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
SKYAI_ADD_MODULE(MAVFOrganizer)
//-------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------
}  // end of avf_organizer_detail
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

