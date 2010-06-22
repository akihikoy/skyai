//-------------------------------------------------------------------------------------------
/*! \file    avf_wire_fitting_smpl.cpp
    \brief   libskyai - wire-fitting function approximator module for action value function
              over continuous state-action space (simple version) (source)
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
#include <skyai/modules_std/avf_wire_fitting_smpl.h>
#include <skyai/modules_std/bits/discaction_selection.h>
#include <lora/stl_ext.h>
#include <lora/stl_math.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
namespace avf_wire_fitting_detail
{
using namespace std;
// using namespace boost;


//===========================================================================================
// class TAVFWireFittingSimpleParameter
//===========================================================================================

#define VCFOR(x_vec,x_itr) \
  for(std::vector<TRealVector>::iterator x_itr(GenBegin(x_vec)); x_itr!=GenEnd(x_vec); ++x_itr)
#define VcCFOR(x_vec,x_itr) \
  for(std::vector<TRealVector>::const_iterator x_itr(GenBegin(x_vec)); x_itr!=GenEnd(x_vec); ++x_itr)
#define VC2FOR(x_vec,x_itr, x_constvec2,x_constitr2) \
  if(x_vec.size()!=x_constvec2.size())  \
    {LERROR("size nonconformant in vector operation: " #x_vec ".size()= "  \
      <<(x_vec).size()<<", " #x_constvec2 ".size()= "<<(x_constvec2).size()); lexit(df);}  \
  std::vector<TRealVector>::const_iterator  x_constitr2(GenBegin(x_constvec2));  \
  for(std::vector<TRealVector>::iterator x_itr(GenBegin(x_vec)); x_itr!=GenEnd(x_vec); ++x_itr, ++x_constitr2)

//!\brief assign zero (but size is not changed)
override void TAVFWireFittingSimpleParameter::Zero (void)
{
  VCFOR(Theta,itr)  {SetZero(*itr);}
  VCFOR(CtrlVec,itr)  {SetZero(*itr);}
}
//-------------------------------------------------------------------------------------------

override TReal TAVFWireFittingSimpleParameter::Norm (void) const
{
  TReal n(0.0l);
  VcCFOR(Theta,itr)   {n+= GetNormSq(*itr);}
  VcCFOR(CtrlVec,itr)  {n+= GetNormSq(*itr);}
  return real_sqrt(n);
}
//-------------------------------------------------------------------------------------------

/*!\brief return (*this = rhs) */
override const TActionValueFuncParamInterface&
    TAVFWireFittingSimpleParameter::operator= (const TActionValueFuncParamInterface &rhs)
{
  const TAVFWireFittingSimpleParameter *prhs= dynamic_cast<const TAVFWireFittingSimpleParameter *>(&rhs);
  if (prhs==NULL)  {LERROR("type conversion error in TAVFWireFittingSimpleParameter"); lexit(df);}

  Theta.resize(prhs->Theta.size());
  CtrlVec.resize(prhs->CtrlVec.size());
  {VC2FOR(Theta,itrl, prhs->Theta,itrr)      {CopyOctArray(*itrl,*itrr);}}
  {VC2FOR(CtrlVec,itrl, prhs->CtrlVec,itrr)    {CopyOctArray(*itrl,*itrr);}}
  return *this;
}
//-------------------------------------------------------------------------------------------

/*!\brief return (*this += rhs) */
override const TActionValueFuncParamInterface&
    TAVFWireFittingSimpleParameter::operator+= (const TActionValueFuncParamInterface &rhs)
{
  const TAVFWireFittingSimpleParameter *prhs= dynamic_cast<const TAVFWireFittingSimpleParameter *>(&rhs);
  if (prhs==NULL)  {LERROR("type conversion error in TAVFWireFittingSimpleParameter"); lexit(df);}

  {VC2FOR(Theta,itrl, prhs->Theta,itrr)      {(*itrl)+= (*itrr);}}
  {VC2FOR(CtrlVec,itrl, prhs->CtrlVec,itrr)    {(*itrl)+= (*itrr);}}
  return *this;
}
//-------------------------------------------------------------------------------------------

//!\brief return (*this += weight*rhs)
override const TActionValueFuncParamInterface&
    TAVFWireFittingSimpleParameter::AddProd (const TReal &weight, const TActionValueFuncParamInterface &rhs)
{
  const TAVFWireFittingSimpleParameter *prhs= dynamic_cast<const TAVFWireFittingSimpleParameter *>(&rhs);
  if (prhs==NULL)  {LERROR("type conversion error in TAVFWireFittingSimpleParameter"); lexit(df);}

  {VC2FOR(Theta,itrl, prhs->Theta,itrr)      {loco_rabbits::WeightedAdd(*itrl, weight, *itrr);}}
  {VC2FOR(CtrlVec,itrl, prhs->CtrlVec,itrr)    {loco_rabbits::WeightedAdd(*itrl, weight, *itrr);}}
  return *this;
}
//-------------------------------------------------------------------------------------------

/*!\brief return (*this *= rhs) */
override const TActionValueFuncParamInterface&
    TAVFWireFittingSimpleParameter::operator*= (const TReal &rhs)
{
  VCFOR(Theta,itr)   {(*itr)*= rhs;}
  VCFOR(CtrlVec,itr)  {(*itr)*= rhs;}
  return *this;
}
//-------------------------------------------------------------------------------------------

#undef VCFOR
#undef VcCFOR
#undef VC2FOR



//===========================================================================================
// class MAVFWireFittingSimple
//===========================================================================================

override void MAVFWireFittingSimple::slot_initialize_exec (void)
{
  LASSERT1op1(conf_.ActionMax.length(),==,conf_.ActionMin.length());

  if (param_.BFSize()*param_.CtrlDim()*param_.WireSize()==0)
  {
    // parameter initialization...

    param_.Init (get_feature_dim(), conf_.ActionDim, conf_.WireSize);

    if (param_.BFSize()*param_.CtrlDim()*param_.WireSize()==0)
    {
      LERROR("invalid parameter vector.");
      LDBGVAR(get_feature_dim()); LDBGVAR(conf_.ActionDim); LDBGVAR(conf_.WireSize);
      lexit(df);
    }

    if (conf_.ActionMax.length()==param_.CtrlDim())
    {
      for (std::vector<TRealVector>::iterator desitr (GenBegin(param_.CtrlVec)); desitr!=GenEnd(param_.CtrlVec); ++desitr)
      {
        const double *amaxptr(OctBegin(conf_.ActionMax)), *aminptr(OctBegin(conf_.ActionMin));
        for (double *qptr(OctBegin(*desitr)); qptr!=OctEnd(*desitr); ++qptr,++amaxptr,++aminptr)
          *qptr = Rand (*aminptr, *amaxptr);
      }
    }
    else if (conf_.ActionMax.length()==0 || conf_.ActionMax.length()==1)
    {
      double amax= (conf_.ActionMax.length()==0 ? 1.0  : conf_.ActionMax(1));
      double amin= (conf_.ActionMin.length()==0 ? -1.0 : conf_.ActionMin(1));
      for (std::vector<TRealVector>::iterator desitr (GenBegin(param_.CtrlVec)); desitr!=GenEnd(param_.CtrlVec); ++desitr)
      {
        for (double *qptr(OctBegin(*desitr)); qptr!=OctEnd(*desitr); ++qptr)
          *qptr = Rand (amin, amax);
      }
    }
    else
      {LERROR("dim of conf_.ActionMax should be in {0,1,"<<param_.CtrlDim()<<"}"); lexit(df);}
  }

  policy.resize (param_.WireSize());

  init_gauss_noise();
}
//-------------------------------------------------------------------------------------------

override void MAVFWireFittingSimple::slot_reset_exec (void)
{
}
//-------------------------------------------------------------------------------------------

override void MAVFWireFittingSimple::slot_add_to_parameter_exec (const TParameter &diff)
{
  param_+= diff;

  // constrain the parameters
  if (conf_.ConstraintKind==sckMinMax)
  {
    apply_min_max_constraint_to_des_i();
  }
  else if (conf_.ConstraintKind!=sckNone)
    {LERROR("invalid conf_.ConstraintKind: "<<(int)conf_.ConstraintKind); lexit(df);}
}
//-------------------------------------------------------------------------------------------


override const MAVFWireFittingSimple::TParameter&
    MAVFWireFittingSimple::out_parameter_ref_get (void) const
{
  return param_;
}
//-------------------------------------------------------------------------------------------

override void MAVFWireFittingSimple::out_parameter_val_get (TParameter &outerparam) const
{
  outerparam= param_;
}
//-------------------------------------------------------------------------------------------

override void MAVFWireFittingSimple::out_evaluate_get (const TState &x, const TAction &a, TStateActionAttribute attrib) const
{
  if (attrib.IsNull())  return;

  /// update cache (1)
  cache.phi= x;
  cache_update_phi();

  cache.a= a;

  ////////////////////////////////////////////////////////////////////////////
  if (attrib.ActionValue==NULL && attrib.StateValue==NULL && attrib.Gradient==NULL)  return;

  /// update cache (2); related to action
  cache_update_a();

  if (attrib.ActionValue)  cache_calc_action_value (*attrib.ActionValue);
  if (attrib.StateValue)   *attrib.StateValue= cache.max_q;

  ////////////////////////////////////////////////////////////////////////////
  if (attrib.Gradient==NULL)  return;
  TAVFWireFittingSimpleParameter *grad= dynamic_cast<TAVFWireFittingSimpleParameter *>(attrib.Gradient);
  if (grad==NULL)  {LERROR("type conversion error in MAVFWireFittingSimple"); lexit(df);}

  /// gradient
  cache_calc_gradient (*grad);

  // TReal norm= grad->norm();
  // if (norm > conf_.GradientMax)  (*grad)*= conf_.GradientMax/norm;
}
//-------------------------------------------------------------------------------------------

override void MAVFWireFittingSimple::out_greedy_get (const TState &x, TAction *greedy, TStateActionAttribute attrib) const
{
  if (greedy==NULL && attrib.IsNull())  return;

  /// update cache (1)
  cache.phi= x;
  cache_update_phi();

  /// select action
  cache.a= cache.u_i[cache.max_q_idx];

  if (greedy)  *greedy= cache.a;

  ////////////////////////////////////////////////////////////////////////////
  if (attrib.ActionValue==NULL && attrib.StateValue==NULL && attrib.Gradient==NULL)  return;

  /// update cache (2); related to action
  cache_update_a();

  if (attrib.ActionValue)  *attrib.ActionValue= cache.max_q;
  if (attrib.StateValue)   *attrib.StateValue= cache.max_q;

  ////////////////////////////////////////////////////////////////////////////
  if (attrib.Gradient==NULL)  return;
  TAVFWireFittingSimpleParameter *grad= dynamic_cast<TAVFWireFittingSimpleParameter *>(attrib.Gradient);
  if (grad==NULL)  {LERROR("type conversion error in MAVFWireFittingSimple"); lexit(df);}

  /// gradient
  cache_calc_gradient (*grad);

  // TReal norm= grad->norm();
  // if (norm > conf_.GradientMax)  (*grad)*= conf_.GradientMax/norm;
}
//-------------------------------------------------------------------------------------------

override void MAVFWireFittingSimple::out_select_action_get (TAction *a, TStateActionAttribute attrib) const
{
  if (a==NULL && attrib.IsNull())  return;

  /// update cache (1)
  cache.phi= get_feature();
  cache_update_phi();

  /// select action
  bool is_greedy (false);
  cache_select_action (conf_.ActionSelection, &is_greedy);

  if (a)  *a= cache.a;

  ////////////////////////////////////////////////////////////////////////////
  if (attrib.ActionValue==NULL && attrib.StateValue==NULL && attrib.Gradient==NULL)  return;

  /// update cache (2); related to action
  cache_update_a();

  if (attrib.ActionValue)
  {
    if (is_greedy)  *attrib.ActionValue= cache.max_q;
    else            cache_calc_action_value (*attrib.ActionValue);
  }
  if (attrib.StateValue)  *attrib.StateValue= cache.max_q;

  ////////////////////////////////////////////////////////////////////////////
  if (attrib.Gradient==NULL)  return;
  TAVFWireFittingSimpleParameter *grad= dynamic_cast<TAVFWireFittingSimpleParameter *>(attrib.Gradient);
  if (grad==NULL)  {LERROR("type conversion error in MAVFWireFittingSimple"); lexit(df);}

  /// gradient
  cache_calc_gradient (*grad);

  // TReal norm= grad->norm();
  // if (norm > conf_.GradientMax)  (*grad)*= conf_.GradientMax/norm;
}
//-------------------------------------------------------------------------------------------

//!\warning Replacing eligibility trace for wire fitting is very experimental! We recommend not to use it.
override void MAVFWireFittingSimple::out_replacing_trace_get (TParameter &eligibility_trace) const
{
  TAVFWireFittingSimpleParameter *p_eligibility_trace
      = dynamic_cast<TAVFWireFittingSimpleParameter *>(&eligibility_trace);
  if (p_eligibility_trace==NULL)
    {LERROR("type conversion error in TAVFWireFittingSimpleParameter"); lexit(df);}

  if (conf_.TraceMax.size()==1)
  {
#if 0
    TReal norm;
    const TReal &TraceMax (conf_.TraceMax[0]);
    for (std::vector<TRealVector>::iterator  vqitr (GenBegin(p_eligibility_trace->Theta));
            vqitr!=GenEnd(p_eligibility_trace->Theta); ++vqitr)
      for (double *first(GenBegin(*vqitr)),*const last(GenEnd(*vqitr)); first!=last; ++first)
        if ((norm=real_fabs(*first)) > TraceMax)  {/*LDBGVAR(*first);*/ (*first)*= TraceMax/norm;}

    for (std::vector<TRealVector>::iterator  desitr (GenBegin(p_eligibility_trace->CtrlVec));
            desitr!=GenEnd(p_eligibility_trace->CtrlVec); ++desitr)
      for (double *first(GenBegin(*desitr)),*const last(GenEnd(*desitr)); first!=last; ++first)
        if ((norm=real_fabs(*first)) > TraceMax)  {/*LDBGVAR(*first);*/ (*first)*= TraceMax/norm;}
#endif
    /*!
      When we applied replacing eligibility trace for every parameters,
      learning become unstable.
      See discussion in "XI. Conclusions"
        of J. N. Tsitsiklis and B. V. Roy, "An analysis of temporal-difference
            learning with function approximation," IEEE Trans on Automatic Control,
            vol. 42, no. 5, pp. 674-690, 1997.
      Thus, here we apply replacing trace only for Theta
      (which is a sum of a feature vector, so physical meaning is close to
      a linear function approximator case)
    */
#if 1
    TReal norm;
    const TReal &TraceMax (conf_.TraceMax[0]);
    for (std::vector<TRealVector>::iterator  vqitr (GenBegin(p_eligibility_trace->Theta));
            vqitr!=GenEnd(p_eligibility_trace->Theta); ++vqitr)
      for (double *first(GenBegin(*vqitr)),*const last(GenEnd(*vqitr)); first!=last; ++first)
        if ((norm=real_fabs(*first)) > TraceMax)  {/*LDBGVAR(*first);*/ (*first)*= TraceMax/norm;}
#endif
  }
  else
  {
    LERROR("error: out_replacing_trace_get is not implemented for conf_.TraceMax.size()!=1");
    lexit(df);
  }
}
//-------------------------------------------------------------------------------------------


override MAVFWireFittingSimple::TParameter*
    MAVFWireFittingSimple::out_create_parameter_get (void) const
{
  TAVFWireFittingSimpleParameter *p = new TAVFWireFittingSimpleParameter();
  if (param_.BFSize()*param_.CtrlDim()*param_.WireSize()==0)
    p->Init (get_feature_dim(), conf_.ActionDim, conf_.WireSize);
  else
    p->Init (param_.BFSize(), param_.CtrlDim(), param_.WireSize());
  return p;
}
//-------------------------------------------------------------------------------------------

override void MAVFWireFittingSimple::out_zero_parameter_get (TParameter &outerparam) const
{
  TAVFWireFittingSimpleParameter *prhs= dynamic_cast<TAVFWireFittingSimpleParameter *>(&outerparam);
  if (prhs==NULL)  {LERROR("type conversion error in TAVFWireFittingSimpleParameter"); lexit(df);}
  prhs->Zero();
}
//-------------------------------------------------------------------------------------------


void MAVFWireFittingSimple::init_gauss_noise ()
{
  /*TEST VERSION*/{
    ColumnVector  sqdiff (param_.CtrlDim());
    if (conf_.ActionMax.length()==0)                      sqdiff.fill (0.0);
    else if (conf_.ActionMax.length()==param_.CtrlDim())  sqdiff= conf_.ActionMax-conf_.ActionMin;
    else                                                   sqdiff.fill (conf_.ActionMax(1)-conf_.ActionMin(1));
    for (double *itr(OctBegin(sqdiff)), *const end(OctEnd(sqdiff)); itr!=end; ++itr)
    {
      *itr= Square(0.5*conf_.NoiseFactor) * Square(*itr);
      if (*itr<conf_.MinimumNoiseVar)  *itr= conf_.MinimumNoiseVar;
    }
    gauss_noise.SetCov (Matrix(DiagMatrix(sqdiff)));
  }//*/
}
//-------------------------------------------------------------------------------------------

//! update cache for cache.phi \note to execute this routine, assign cache.phi
void MAVFWireFittingSimple::cache_update_phi () const
{
  // FIXME inefficient computation
  // update cache.u_i
  {
    if (static_cast<int>(cache.u_i.size()) != param_.WireSize())
    {
      cache.u_i.resize (param_.WireSize());
    }
    std::vector<TRealVector>::const_iterator  desitr (GenBegin(param_.CtrlVec));
    for(std::vector<TRealVector>::iterator uitr(GenBegin(cache.u_i)); uitr!=GenEnd(cache.u_i); ++uitr,++desitr)
    {
      *uitr= (*desitr);
    }
  }
  // update cache.q_i & cache.max_{q,q_idx}
  {
    if (static_cast<int>(cache.q_i.size()) != param_.WireSize())
      cache.q_i.resize (param_.WireSize());
    std::vector<TRealVector>::const_iterator  vqitr (GenBegin(param_.Theta));
    for(std::vector<TValue>::iterator qitr(GenBegin(cache.q_i)); qitr!=GenEnd(cache.q_i); ++qitr,++vqitr)
      *qitr= InnerProd(GenBegin(*vqitr),GenEnd(*vqitr),GenBegin(cache.phi));
    cache.max_q_idx= max_element_index (cache.q_i.begin(), cache.q_i.end());
    cache.max_q= cache.q_i[cache.max_q_idx];
  }
}
//-------------------------------------------------------------------------------------------

//! update cache for cache.a \note to execute this routine, assign cache.a and execute cache_update_phi
void MAVFWireFittingSimple::cache_update_a () const
{
  // update cache.D_i
  {
    if (static_cast<int>(cache.D_i.size()) != param_.WireSize())
      cache.D_i.resize (param_.WireSize());
    std::vector<TRealVector>::const_iterator uitr(cache.u_i.begin());
    std::vector<TValue>::const_iterator      qitr(cache.q_i.begin());
    for(std::vector<TValue>::iterator Ditr(cache.D_i.begin()); Ditr!=cache.D_i.end(); ++Ditr,++uitr,++qitr)
      *Ditr= GetNormSq(cache.a - *uitr) + conf_.SmoothingFactor * (cache.max_q - *qitr) + conf_.Tiny;
  }
}
//-------------------------------------------------------------------------------------------

//! select action from cache data and store it into  cache.a
void MAVFWireFittingSimple::cache_select_action (TAVFWFSmplActionSelection ActionSelection, bool *res_is_greedy) const
{
  bool is_greedy (false);
  if (ActionSelection == asGreedy
      || ActionSelection == asEpsGreedy)
  {
    /*select greedy or random*/{
      if (ActionSelection == asGreedy)
        is_greedy= true;
      else
      {
        TReal eps= get_eps();
        if (RollADice(1.0l-eps))  is_greedy= true;
        else is_greedy= false;
      }
    }
    /*action selection*/{
      if (is_greedy)
      {
        cache.a= cache.u_i[cache.max_q_idx];
      }
      else // randomly decide cache.a
      {
        cache.a.resize (param_.CtrlDim());
        const double *amaxptr(OctBegin(conf_.ActionMax)), *aminptr(OctBegin(conf_.ActionMin));
        for (double *aptr(OctBegin(cache.a)); aptr!=OctEnd(cache.a); ++aptr,++amaxptr,++aminptr)
          *aptr = Rand (*aminptr, *amaxptr);
      }
    }
  }
  else if (ActionSelection == asWFBoltzman)
  {
    // select cache.u_i (i=i_wire) by boltzmann selection with cache.q_i
    TReal tau= get_tau();
    disc_action::Boltzmann (tau, cache.q_i.begin(), cache.q_i.end(), OctBegin(policy), OctEnd(policy));
    int i_wire= disc_action::SelectActionFromPolicy(OctBegin(policy),OctEnd(policy));
    // generate an action
    cache.a= cache.u_i[i_wire];
    loco_rabbits::WeightedAdd (cache.a, 1.0-policy(i_wire), gauss_noise());
  }
  else
    {LERROR("invalid ActionSelection: "<<static_cast<int>(ActionSelection)); lexit(df);}

  if (res_is_greedy)  (*res_is_greedy)= is_greedy;
}
//-------------------------------------------------------------------------------------------

//! calculate action value q from cache \note REQUIRED: cache_update_phi , cache_update_a
void MAVFWireFittingSimple::cache_calc_action_value (TReal &q) const
{
  TValue  invDsum(0.0l), invDQsum(0.0l);  // TODO these are also used in cache_calc_gradient, so cache them for more efficiency
  for (std::vector<TValue>::const_iterator qitr(cache.q_i.begin()), Ditr(cache.D_i.begin()); qitr!=cache.q_i.end(); ++qitr,++Ditr)
  {
    invDQsum+= (*qitr)/(*Ditr);
    invDsum += 1.0l/(*Ditr);
  }

  q= invDQsum / invDsum;
}
//-------------------------------------------------------------------------------------------

//! calculate gradient grad from cache \note REQUIRED: cache_update_phi , cache_update_a
void MAVFWireFittingSimple::cache_calc_gradient (TAVFWireFittingSimpleParameter &grad) const
{
  TValue  invDsum(0.0l), invD2sum(0.0l), invDQsum(0.0l), invD2Qsum(0.0l), tmp;
  for (std::vector<TValue>::const_iterator qitr(GenBegin(cache.q_i)), Ditr(GenBegin(cache.D_i)); qitr!=GenEnd(cache.q_i); ++qitr,++Ditr)
  {
    tmp      = (*qitr)/(*Ditr);
    invDQsum += tmp;
    invD2Qsum+= tmp/(*Ditr);
    invDsum  += 1.0l/(*Ditr);
    invD2sum += 1.0l/Square(*Ditr);
  }

  std::vector<TRealVector>::const_iterator     desitr (GenBegin(param_.CtrlVec));
  // std::vector<TRealVector>::const_iterator  vqitr (GenBegin(param_.Theta));
  std::vector<TRealVector>::const_iterator     uitr(GenBegin(cache.u_i));
  std::vector<TValue>::const_iterator          qitr(GenBegin(cache.q_i));
  std::vector<TValue>::const_iterator          Ditr(GenBegin(cache.D_i));

  // FIXME inefficient computation (prevent the memory re-allocation(s))
  grad.Init (param_.BFSize(), param_.CtrlDim(), param_.WireSize());
  std::vector<TRealVector>::iterator           grad_desitr (GenBegin(grad.CtrlVec));
  std::vector<TRealVector>::iterator           grad_vqitr (GenBegin(grad.Theta));
  for (int j(0), wc(param_.WireSize()); j<wc; ++j)
  {
    TValue tmp(0.0l);
    tmp+= conf_.SmoothingFactor / invDsum * ((*qitr)/Square(*Ditr) - (j==cache.max_q_idx?invD2Qsum:0.0l));
    tmp+= 1.0l/(*Ditr)/invDsum;
    tmp+= conf_.SmoothingFactor / Square(invDsum) * invDQsum * ((j==cache.max_q_idx?invD2sum:0.0l) - 1.0l/Square(*Ditr));
    *grad_vqitr = tmp * cache.phi;

    tmp = 2.0l*(invDQsum/Square(invDsum) - (*qitr)/invDsum) / Square(*Ditr);
    *grad_desitr = tmp * (*uitr - cache.a);

    ++desitr; /*++vqitr;*/ ++uitr; ++qitr; ++Ditr;
    ++grad_desitr; ++grad_vqitr;
  }
}
//-------------------------------------------------------------------------------------------

void MAVFWireFittingSimple::apply_min_max_constraint_to_des_i (void)
{
  LASSERT1op1(conf_.ActionMax.length(),>,0);
  LASSERT1op1(conf_.ActionMax.length(),==,conf_.ActionMin.length());

  for (std::vector<TRealVector>::iterator desitr (GenBegin(param_.CtrlVec)); desitr!=GenEnd(param_.CtrlVec); ++desitr)
  {
    if (conf_.ActionMax.length()==param_.CtrlDim())
    {
      const double *amaxptr(GenBegin(conf_.ActionMax)), *aminptr(GenBegin(conf_.ActionMin));
      for (double *qptr(GenBegin(*desitr)); qptr!=GenEnd(*desitr); ++qptr,++amaxptr,++aminptr)
        *qptr = ApplyRange(*qptr, *aminptr, *amaxptr);
    }
    else
    {
      double amax= (conf_.ActionMax.length()==0 ? 1.0  : conf_.ActionMax(1));
      double amin= (conf_.ActionMin.length()==0 ? -1.0 : conf_.ActionMin(1));
      for (double *qptr(GenBegin(*desitr)); qptr!=GenEnd(*desitr); ++qptr)
        *qptr = ApplyRange(*qptr, amin, amax);
    }
  }
}
//-------------------------------------------------------------------------------------------

inline TReal MAVFWireFittingSimple::get_eps () const
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
inline TReal MAVFWireFittingSimple::get_tau () const
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


//-------------------------------------------------------------------------------------------
SKYAI_ADD_MODULE(MAVFWireFittingSimple)
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // end of avf_wire_fitting_detail
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

